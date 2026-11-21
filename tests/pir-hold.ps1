# Exercise the actual decision/state-machine section, with hardware calls stubbed.
# ESP8266 unsigned long is 32-bit; use uint in the C# host test.
$source = Get-Content (Join-Path $PSScriptRoot '..\night_light_fixed.ino') -Raw -Encoding UTF8
$start = $source.IndexOf('  bool blanked =', $source.IndexOf('void updateMotion()'))
$end = $source.IndexOf('// ================= WI-FI: AP', $start)
$core = $source.Substring($start, $end - $start).TrimEnd()
$core = $core.Substring(0, $core.LastIndexOf('}'))
$core = $core.Replace('unsigned long', 'uint').Replace('(long)', '(int)').Replace('UL', 'U')
$dayStart = $source.IndexOf('void updateDayNight(')
$dayEnd = $source.IndexOf('void updateBrightness()', $dayStart)
$dayCode = $source.Substring($dayStart, $dayEnd - $dayStart).Replace('unsigned long', 'uint')
$harness = @'
#pragma warning disable 0649, 0414
using System;
public class PirHoldTest {
  const int ST_OFF=0, ST_ON=1, ST_WAVE=2;
  int lampState, highCount, lampChanges;
  uint now, pirBlankUntil, bootAt, warmupMs, pirHighNowMs, motionUntil;
  uint lampOnAt, waveStart, lastFrame;
  int currentBrightness=100, waveFromBr;
  bool pirPinHigh, pirStuck, pirRunFalse, pirRunBlocked, motionNow, motionPulse;
  bool schedAllowedNow=true, gate, dark;
  string lampWhy;
  class Config { public uint holdSec=10, waveMs=2500, pirMaxRunSec=0; public int windowMin=42, dayLevel=450; }
  Config cfg=new Config();
  bool daylight, dayCandidate;
  uint dayCandidateAt;
  float ldrSmooth;
  const int DAYLIGHT_HYST=60;
  const uint DAYLIGHT_SETTLE_MS=2000;
  int lightLevelFor(float raw) { return (int)raw; }
__DAY__
  const uint PIR_MIN_RUN_MS=300;
  bool pirGated(uint t) { return gate; }
  bool isReallyDark() { return dark; }
  bool timeReached(uint t, uint deadline) { return unchecked((int)(t-deadline))>=0; }
  void pirBlindFor(uint ms) {}
  void lampOffCooldown(uint t) {}
  void startVibe() {}
  void Tick() {
__CORE__
  }
  static void Check(bool ok, string message) { if(!ok) throw new Exception(message); }
  public static string Run() {
    int count=0;
    foreach(uint hold in new uint[]{1,10,30})
    foreach(uint maxRun in new uint[]{0,10})
    foreach(bool rf in new bool[]{false,true})
    foreach(bool stuck in new bool[]{false,true}) {
      var t=new PirHoldTest();
      t.cfg.holdSec=hold; t.cfg.pirMaxRunSec=maxRun;
      t.lampState=ST_ON; t.pirPinHigh=true; t.pirRunFalse=rf; t.pirStuck=stuck;
      t.pirRunBlocked=true; t.gate=true; t.pirBlankUntil=200000;
      // Zero confirmed samples, bright LDR, blocked start: none may cut off active HIGH.
      for(t.now=1000; t.now<=121000; t.now+=100) {
        t.Tick(); Check(t.lampState==ST_ON && t.motionNow,"HIGH lost hold");
      }
      uint lastHigh=121000;
      t.pirPinHigh=false; t.now=lastHigh+hold*1000-1; t.Tick();
      Check(t.lampState==ST_ON,"LOW expired before configured delay");
      t.now++; t.Tick(); Check(t.lampState==ST_WAVE,"LOW did not start fade on deadline");
      t.pirPinHigh=true; t.now++; t.Tick();
      Check(t.lampState==ST_ON,"HIGH did not cancel fade");
      t.schedAllowedNow=false; t.now++; t.Tick();
      Check(t.lampState==ST_WAVE,"HIGH bypassed schedule restriction");
      count++;
    }
    var off=new PirHoldTest(); off.lampState=ST_OFF; off.pirPinHigh=true;
    off.pirRunFalse=true; off.highCount=50; off.pirHighNowMs=500; off.dark=true;
    off.Tick(); Check(off.lampState==ST_OFF,"RF filter no longer protects first start");
    foreach(int state in new int[]{ST_OFF,ST_ON,ST_WAVE}) {
      var t=new PirHoldTest(); t.lampState=state; t.ldrSmooth=900;
      t.updateDayNight(0); t.updateDayNight(1999);
      Check(!t.daylight,"Day detection ignored settle delay");
      t.updateDayNight(2000);
      Check(t.daylight,"Day detection failed in state "+state);
      t.ldrSmooth=420; t.updateDayNight(3000); t.updateDayNight(6000);
      Check(t.daylight,"Day hysteresis failed");
      t.ldrSmooth=0; t.updateDayNight(7000); t.updateDayNight(8999);
      Check(t.daylight,"Night detection ignored settle delay");
      t.updateDayNight(9000);
      Check(!t.daylight,"Night detection failed in state "+state);
      t.ldrSmooth=900; t.updateDayNight(10000);
      t.ldrSmooth=0; t.updateDayNight(11000);
      t.ldrSmooth=900; t.updateDayNight(12000); t.updateDayNight(13999);
      Check(!t.daylight,"Brief light change was not discarded");
    }
    return "PASS: "+count+" hold scenarios + first-start RF filtering + day/night, hysteresis and settle delay (OFF/ON/WAVE)";
  }
}
'@
Add-Type -TypeDefinition ($harness.Replace('__CORE__', $core).Replace('__DAY__', $dayCode)) -ErrorAction Stop
[PirHoldTest]::Run()
