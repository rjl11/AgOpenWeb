// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using System;
using AgOpenWeb.Models;
using AgOpenWeb.Models.Base;
using AgOpenWeb.Models.Configuration;
using AgOpenWeb.Services.AutoSteer;
using AgOpenWeb.Services.Interfaces;
using AgOpenWeb.Services.Tool;
using NSubstitute;

namespace AgOpenWeb.Services.Tests.Tool;

/// <summary>
/// A hitch angle sensor (PGN 218, Docs/PGN.md): the packet, its way from the module port to
/// the tool service, and the trailing tool placed at the measured angle while the reading is
/// fresh and valid, the estimate otherwise. Positive is the implement to the right of the
/// tractor's line, seen from the seat.
/// </summary>
[TestFixture]
public class HitchAngleSensorTests
{
    private static byte[] Packet(double degrees, bool valid = true)
    {
        short v = (short)Math.Round(degrees * 100);
        var p = new byte[] { 0x80, 0x81, 0x7F, PgnNumbers.HITCH_ANGLE, 8,
            (byte)(v & 0xFF), (byte)((v >> 8) & 0xFF), (byte)(valid ? 1 : 0), 0, 0, 0, 0, 0, 0 };
        byte crc = 0;
        for (int i = 2; i < p.Length - 1; i++) crc += p[i];
        p[^1] = crc;
        return p;
    }

    [TestCase(12.34, true)]
    [TestCase(-45.5, true)]
    [TestCase(0.0, false)]
    public void Parse_ReadsAngleAndValidFlag(double degrees, bool valid)
    {
        Assert.That(PgnBuilder.TryParseHitchAngle(Packet(degrees, valid), out double got, out bool gotValid), Is.True);
        Assert.That(got, Is.EqualTo(degrees).Within(0.005));
        Assert.That(gotValid, Is.EqualTo(valid));
    }

    [Test]
    public void Parse_RejectsWrongLengthOrPgn()
    {
        var p = Packet(10);
        Assert.That(PgnBuilder.TryParseHitchAngle(p.AsSpan(0, 13), out _, out _), Is.False);
        p[3] = PgnNumbers.IMU_DATA;
        Assert.That(PgnBuilder.TryParseHitchAngle(p, out _, out _), Is.False);
    }

    [Test]
    public void ModulePort_RaisesHitchAngleReceived()
    {
        var udp = Substitute.For<IUdpCommunicationService>();
        var steer = new AutoSteerService(Substitute.For<ITrackGuidanceService>(), udp,
            Substitute.For<IGpsService>(), new ConfigurationStore());
        steer.Start();
        try
        {
            (double deg, bool valid)? got = null;
            steer.HitchAngleReceived += (d, v) => got = (d, v);
            udp.DataReceived += Raise.Event<EventHandler<UdpDataReceivedEventArgs>>(udp,
                new UdpDataReceivedEventArgs { Data = Packet(-7.25), PGN = PgnNumbers.HITCH_ANGLE });
            Assert.That(got, Is.Not.Null);
            Assert.That(got!.Value.deg, Is.EqualTo(-7.25).Within(0.005));
            Assert.That(got.Value.valid, Is.True);
        }
        finally { steer.Stop(); }
    }

    private long _nowMs;

    private (ToolPositionService svc, ConfigurationStore cfg) Trailing(bool sensorOn = true)
    {
        var cfg = new ConfigurationStore();
        cfg.Tool.IsToolTrailing = true;
        cfg.Tool.IsToolTBT = false;
        cfg.Tool.IsToolRearFixed = false;
        cfg.Tool.IsToolFrontFixed = false;
        cfg.Tool.TrailingHitchLength = 4.0;
        cfg.Tool.TrailingToolToPivotLength = 0;
        cfg.Tool.UseHitchAngleSensor = sensorOn;
        _nowMs = 1_000_000;
        var svc = new ToolPositionService(cfg) { TickClock = () => _nowMs };
        return (svc, cfg);
    }

    /// <summary>Drive straight north, 10 ms ticks at 3.6 km/h, past the startup snap.</summary>
    private void DriveNorth(ToolPositionService svc, int ticks = 80, Action? eachTick = null)
    {
        for (int i = 0; i < ticks; i++)
        {
            eachTick?.Invoke();
            svc.Update(new Vec3(0, i * 0.01, 0), 0);
            _nowMs += 10;
        }
    }

    private static double Deg(double rad) => rad * 180.0 / Math.PI;

    [Test]
    public void FreshReading_PlacesToolAtMeasuredAngle_ToTheRight()
    {
        var (svc, _) = Trailing();
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(10, true));

        Assert.That(svc.IsHitchAngleMeasured, Is.True);
        Assert.That(Deg(svc.HitchAngle), Is.EqualTo(10).Within(1e-6));
        // Heading north, right is east: the tool sits east of the hitch, 4 m back along 10°.
        var hitch = svc.HitchPosition;
        Assert.That(svc.ToolPosition.Easting - hitch.Easting, Is.EqualTo(4.0 * Math.Sin(10 * Math.PI / 180)).Within(1e-6));
        Assert.That(hitch.Northing - svc.ToolPosition.Northing, Is.EqualTo(4.0 * Math.Cos(10 * Math.PI / 180)).Within(1e-6));
    }

    [Test]
    public void SensorOff_UsesTheEstimate()
    {
        var (svc, _) = Trailing(sensorOn: false);
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(10, true));

        Assert.That(svc.IsHitchAngleMeasured, Is.False);
        Assert.That(Deg(svc.HitchAngle), Is.EqualTo(0).Within(0.01)); // straight behind on a straight
        Assert.That(svc.LastHitchSensorReading, Is.Not.Null, "the reading is still shown for setting up");
    }

    [Test]
    public void StaleOrInvalidReading_FallsBackToTheEstimate()
    {
        var (svc, _) = Trailing();
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(10, true));
        Assert.That(svc.IsHitchAngleMeasured, Is.True);

        _nowMs += ToolPositionService.HitchSensorFreshMs;
        svc.Update(new Vec3(0, 0.8, 0), 0);
        Assert.That(svc.IsHitchAngleMeasured, Is.False, "older than 300 ms");
        Assert.That(svc.LastHitchSensorReading!.Value.Fresh, Is.False);

        svc.SetMeasuredHitchAngle(10, valid: false);
        svc.Update(new Vec3(0, 0.81, 0), 0);
        Assert.That(svc.IsHitchAngleMeasured, Is.False, "flagged not valid");
    }

    [Test]
    public void InvertAndZeroOffset_AreApplied()
    {
        var (svc, cfg) = Trailing();
        cfg.Tool.HitchAngleInvert = true;
        cfg.Tool.HitchAngleOffset = 2.0;
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(-12, true));

        Assert.That(Deg(svc.HitchAngle), Is.EqualTo(10).Within(1e-6)); // -(-12) - 2
    }

    [Test]
    public void LargeMeasuredAngle_IsKept_NoJackknifeSnap()
    {
        var (svc, _) = Trailing();
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(-150, true));

        Assert.That(svc.IsHitchAngleMeasured, Is.True);
        Assert.That(Deg(svc.HitchAngle), Is.EqualTo(-150).Within(1e-6));
    }

    [Test]
    public void Tbt_IgnoresTheSensor()
    {
        var (svc, cfg) = Trailing();
        cfg.Tool.IsToolTBT = true;
        DriveNorth(svc, eachTick: () => svc.SetMeasuredHitchAngle(10, true));

        Assert.That(svc.IsHitchAngleMeasured, Is.False);
        Assert.That(Deg(svc.HitchAngle), Is.EqualTo(0).Within(0.01));
    }

    [TestCase(0.1, 6.2, 0.1 - 6.2 + 2 * Math.PI)]
    [TestCase(6.2, 0.1, 6.1 - 2 * Math.PI)]
    [TestCase(1.0, 0.5, 0.5)]
    public void HitchAngleFrom_WrapsToPlusMinusPi(double vehicle, double tool, double expected)
    {
        Assert.That(ToolPositionService.HitchAngleFrom(vehicle, tool), Is.EqualTo(expected).Within(1e-9));
    }
}
