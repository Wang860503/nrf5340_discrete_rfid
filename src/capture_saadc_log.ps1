param(
    [string]$Port = "COM16",
    [int]$Baud = 115200,
    [int]$Seconds = 90,
    [string]$OutFile = "saadc_autotest_$(Get-Date -Format 'yyyyMMdd_HHmmss').log"
)

$ErrorActionPreference = "Stop"
$sw = [Diagnostics.Stopwatch]::StartNew()

function Get-MetricFromLine {
    param([string]$Line)

    $m = [orderedhashtable]@{}
    if ($Line -match 'abs_avg=(\d+)') { $m.abs_avg = [int]$Matches[1] }
    if ($Line -match 'clip=(\d+)') { $m.clip = [int]$Matches[1] }
    if ($Line -match 'clip=(\d+)/(\d+)') {
        $m.clip = [int]$Matches[1]
        $m.clip_total = [int]$Matches[2]
    }
    if ($Line -match 'bad=(\d+)') { $m.bad = [int]$Matches[1] }
    if ($Line -match 'header=(\d+)') { $m.header = [int]$Matches[1] }
    if ($Line -match 'row=(\d+)') { $m.row = [int]$Matches[1] }
    if ($Line -match 'col=(\d+)') { $m.col = [int]$Matches[1] }
    if ($Line -match 'stop=(\d+)') { $m.stop = [int]$Matches[1] }
    if ($Line -match 'half_q8=(\d+)') { $m.half_q8 = [int]$Matches[1] }
    if ($Line -match 'SUCCESS-SAADC') { $m.success = $true }
    if ($Line -match 'No card detected') { $m.no_card = $true }
    if ($Line -match 'saadc_card_gate_skip') { $m.gate_skip = $true }
    if ($Line -match 'CARD-PRESENT-SAADC') { $m.card_present_warn = $true }
    if ($Line -match 'strict_pair_fail') { $m.strict_fail = $true }
    return $m
}

try {
    $port = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
    $port.ReadTimeout = 200
    $port.DtrEnable = $true
    $port.RtsEnable = $true
    $port.Open()

    Write-Host "Capturing $Port -> $OutFile for $Seconds s..."
    $reader = $port.BaseStream
    $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)

    while ([DateTime]::UtcNow -lt $deadline) {
        if ($reader.BytesToRead -gt 0) {
            $chunk = $reader.Read($reader.BytesToRead)
            if ($chunk.Length -gt 0) {
                $text = [Text.Encoding]::UTF8.GetString($chunk)
                Add-Content -Path $OutFile -Value $text -Encoding UTF8
            }
        } else {
            Start-Sleep -Milliseconds 50
        }
    }
} finally {
    if ($port.IsOpen) { $port.Close() }
}

$lines = Get-Content $OutFile -ErrorAction SilentlyContinue
$metrics = @()
$successCount = 0
$noCardCount = 0
$gateSkipCount = 0
$presentCount = 0

foreach ($line in $lines) {
    if ($line -notmatch 'rfid_main:') { continue }
    $m = Get-MetricFromLine $line
    if ($m.Count -eq 0) { continue }

    if ($m.success) { $successCount++ }
    if ($m.no_card) { $noCardCount++ }
    if ($m.gate_skip) { $gateSkipCount++ }
    if ($m.card_present_warn) { $presentCount++ }

    $metrics += ,[PSCustomObject]$m
}

$clipRates = $metrics | Where-Object { $_.clip_total -and $_.clip_total -gt 0 } |
    ForEach-Object { [math]::Round(100.0 * $_.clip / $_.clip_total, 2) }

Write-Host ""
Write-Host "=== SAADC Auto Capture Summary ==="
Write-Host "File: $OutFile"
Write-Host "Duration: $Seconds s"
Write-Host "Success: $successCount"
Write-Host "No card: $noCardCount"
Write-Host "Gate skip: $gateSkipCount"
Write-Host "Card-present warn: $presentCount"
if ($metrics.Count -gt 0) {
    $absAvg = ($metrics | Where-Object { $_.abs_avg } | Measure-Object -Property abs_avg -Average).Average
    $badMin = ($metrics | Where-Object { $_.bad } | Measure-Object -Property bad -Minimum).Minimum
    $badMax = ($metrics | Where-Object { $_.bad } | Measure-Object -Property bad -Maximum).Maximum
    $clipMin = ($clipRates | Measure-Object -Minimum).Minimum
    $clipMax = ($clipRates | Measure-Object -Maximum).Maximum
    $clipAvg = ($clipRates | Measure-Object -Average).Average
    Write-Host ("abs_avg avg={0:F0}" -f $absAvg)
    Write-Host ("bad min/max={0}/{1}" -f $badMin, $badMax)
    Write-Host ("clip% min/avg/max={0}/{1}/{2}" -f $clipMin, $clipAvg, $clipMax)
}

$sw.Stop()
Write-Host ("Elapsed: {0:F1}s" -f $sw.Elapsed.TotalSeconds)