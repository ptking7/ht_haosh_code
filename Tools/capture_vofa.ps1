[CmdletBinding()]
param(
    [string]$Port = "COM4",
    [int]$DurationSeconds = 40,
    [string]$OutputDir = "captures",
    [int]$WaitPortSeconds = 15,
    [string[]]$Commands = @()
)

$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Drawing

$floatCount = 10
$packetSize = $floatCount * 4
$tail = [byte[]](0x00, 0x00, 0x80, 0x7f)
$projectRoot = Split-Path -Parent $PSScriptRoot
$outputPath = Join-Path $projectRoot $OutputDir
if (-not (Test-Path -LiteralPath $outputPath)) {
    New-Item -ItemType Directory -Path $outputPath | Out-Null
}

$portDeadline = (Get-Date).AddSeconds($WaitPortSeconds)
$serial = $null
$lastOpenError = $null
while ($null -eq $serial) {
    if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $Port) {
        $candidate = New-Object System.IO.Ports.SerialPort(
            $Port, 115200, "None", 8, "One")
        $candidate.ReadTimeout = 300
        $candidate.DtrEnable = $false
        $candidate.RtsEnable = $false
        try {
            $candidate.Open()
            $serial = $candidate
        }
        catch {
            $lastOpenError = $_.Exception.Message
            $candidate.Dispose()
        }
    }

    if (($null -eq $serial) -and ((Get-Date) -ge $portDeadline)) {
        throw "Unable to open $Port. Last error: $lastOpenError"
    }
    if ($null -eq $serial) {
        Start-Sleep -Milliseconds 250
    }
}

foreach ($command in $Commands) {
    $serial.Write($command + "`r`n")
    Write-Output ("COMMAND=" + $command)
    Start-Sleep -Milliseconds 50
}

$buffer = New-Object "System.Collections.Generic.List[byte]"
$rows = New-Object "System.Collections.Generic.List[object]"
$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
$lastProgress = -10

try {
    while ($stopwatch.Elapsed.TotalSeconds -lt $DurationSeconds) {
        $available = $serial.BytesToRead
        if ($available -gt 0) {
            $bytes = New-Object byte[] $available
            $read = $serial.Read($bytes, 0, $available)
            for ($i = 0; $i -lt $read; $i++) {
                $buffer.Add($bytes[$i])
            }

            while ($true) {
                $tailIndex = -1
                for ($i = 0; $i -le $buffer.Count - 4; $i++) {
                    if (($buffer[$i] -eq $tail[0]) -and
                        ($buffer[$i + 1] -eq $tail[1]) -and
                        ($buffer[$i + 2] -eq $tail[2]) -and
                        ($buffer[$i + 3] -eq $tail[3])) {
                        $tailIndex = $i
                        break
                    }
                }

                if ($tailIndex -lt 0) {
                    if ($buffer.Count -gt 127) {
                        $buffer.RemoveRange(0, $buffer.Count - 35)
                    }
                    break
                }

                if ($tailIndex -ge $packetSize) {
                    $values = New-Object "float[]" $floatCount
                    for ($ch = 0; $ch -lt $floatCount; $ch++) {
                        $valueBytes = $buffer.GetRange(
                            $tailIndex - $packetSize + $ch * 4, 4).ToArray()
                        $values[$ch] = [BitConverter]::ToSingle($valueBytes, 0)
                    }

                    $rows.Add([pscustomobject]@{
                        Time      = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
                        CcrA      = $values[0]
                        CcrB      = $values[1]
                        CcrC      = $values[2]
                        Ia        = $values[3]
                        Ib        = $values[4]
                        Ic        = $values[5]
                        SpeedRef  = $values[6]
                        SpeedFdbk = $values[7]
                        InnerSpeedRef  = $values[8]
                        InnerSpeedFdbk = $values[9]
                    })
                }

                $buffer.RemoveRange(0, $tailIndex + 4)
            }
        }
        else {
            Start-Sleep -Milliseconds 2
        }

        $elapsed = [Math]::Floor($stopwatch.Elapsed.TotalSeconds)
        if (($elapsed -ge $lastProgress + 5) -and ($rows.Count -gt 0)) {
            $last = $rows[$rows.Count - 1]
            Write-Output ("t={0,2}s frames={1,6} out={2,7:F3} inner={3,8:F3} rpm CCR={4,6:F0}/{5,6:F0}/{6,6:F0}" -f
                $elapsed, $rows.Count, $last.SpeedFdbk, $last.InnerSpeedFdbk,
                $last.CcrA, $last.CcrB, $last.CcrC)
            $lastProgress = $elapsed
        }
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
    $serial.Dispose()
}

if ($rows.Count -eq 0) {
    throw "No 8-channel JustFloat frames were received from $Port."
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$csvPath = Join-Path $outputPath "motor_wave_$stamp.csv"
$pngPath = Join-Path $outputPath "motor_wave_$stamp.png"
$latestPngPath = Join-Path $outputPath "motor_wave_latest.png"
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

$imageWidth = 1400
$imageHeight = 1080
$bitmap = New-Object System.Drawing.Bitmap($imageWidth, $imageHeight)
$graphics = [System.Drawing.Graphics]::FromImage($bitmap)
$graphics.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$graphics.Clear([System.Drawing.Color]::White)
$titleFont = New-Object System.Drawing.Font("Segoe UI", 18, [System.Drawing.FontStyle]::Bold)
$panelFont = New-Object System.Drawing.Font("Segoe UI", 12, [System.Drawing.FontStyle]::Bold)
$labelFont = New-Object System.Drawing.Font("Segoe UI", 9)
$smallFont = New-Object System.Drawing.Font("Segoe UI", 8)
$gridPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(225, 225, 225), 1)
$axisPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(80, 80, 80), 1.5)
$textBrush = [System.Drawing.Brushes]::Black

$graphics.DrawString(
    "Speed-loop waveform  $Port  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    $titleFont, $textBrush, 35, 16)

function Get-Range {
    param($Data, $Properties)
    $values = [double[]]@($Data | ForEach-Object {
        foreach ($property in $Properties) { [double]$_.$property }
    })
    $minimum = ($values | Measure-Object -Minimum).Minimum
    $maximum = ($values | Measure-Object -Maximum).Maximum
    if ([Math]::Abs($maximum - $minimum) -lt 1e-6) {
        $maximum += 0.1
        $minimum -= 0.1
    }
    $margin = ($maximum - $minimum) * 0.08
    return @(($minimum - $margin), ($maximum + $margin))
}

function Draw-Panel {
    param($G, $Data, [int]$Left, [int]$Top, [int]$Width, [int]$Height,
          [string]$Title, $Series, $Range)
    $G.DrawString($Title, $panelFont, $textBrush, $Left, ($Top - 24))
    $G.DrawRectangle($axisPen, $Left, $Top, $Width, $Height)

    for ($grid = 0; $grid -le 4; $grid++) {
        $y = $Top + $Height * $grid / 4
        $G.DrawLine($gridPen, $Left, $y, $Left + $Width, $y)
        $value = [double]$Range[1] - ([double]$Range[1] - [double]$Range[0]) * $grid / 4
        $G.DrawString(("{0:F3}" -f $value), $smallFont, $textBrush,
            ($Left - 45), ($y - 8))
    }

    $count = $Data.Count
    $yMin = [double]$Range[0]
    $yMax = [double]$Range[1]
    foreach ($item in $Series) {
        $pen = New-Object System.Drawing.Pen($item.Color, 2)
        $lastPoint = $null
        for ($i = 0; $i -lt $count; $i++) {
            $value = [double]$Data[$i].$($item.Property)
            $x = $Left + $Width * $i / [Math]::Max(1, $count - 1)
            $y = $Top + $Height * ($yMax - $value) / ($yMax - $yMin)
            $point = New-Object System.Drawing.PointF($x, $y)
            if ($null -ne $lastPoint) {
                $G.DrawLine($pen, $lastPoint, $point)
            }
            $lastPoint = $point
        }
        $pen.Dispose()
    }

    $legendX = $Left + 12
    foreach ($item in $Series) {
        $brush = New-Object System.Drawing.SolidBrush($item.Color)
        $G.FillRectangle($brush, $legendX, ($Top + 10), 16, 4)
        $G.DrawString($item.Name, $labelFont, $textBrush,
            ($legendX + 20), ($Top + 3))
        $legendX += 115
        $brush.Dispose()
    }
}

$plotRows = @($rows | ForEach-Object { $_ })
$voltageRange = Get-Range $plotRows @("CcrA", "CcrB", "CcrC")
$currentRange = Get-Range $plotRows @("Ia", "Ib", "Ic")
$speedRange = Get-Range $plotRows @("SpeedRef", "SpeedFdbk")
$innerSpeedRange = Get-Range $plotRows @("InnerSpeedRef", "InnerSpeedFdbk")

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 80 -Width 1270 -Height 210 `
    -Title "SVPWM CCR (counts)" -Range $voltageRange -Series @(
        [pscustomobject]@{ Name = "A"; Property = "CcrA"; Color = [System.Drawing.Color]::RoyalBlue },
        [pscustomobject]@{ Name = "B"; Property = "CcrB"; Color = [System.Drawing.Color]::OrangeRed },
        [pscustomobject]@{ Name = "C"; Property = "CcrC"; Color = [System.Drawing.Color]::DarkGreen }
    )

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 345 -Width 1270 -Height 210 `
    -Title "Phase current (A)" -Range $currentRange -Series @(
        [pscustomobject]@{ Name = "Ia"; Property = "Ia"; Color = [System.Drawing.Color]::RoyalBlue },
        [pscustomobject]@{ Name = "Ib"; Property = "Ib"; Color = [System.Drawing.Color]::OrangeRed },
        [pscustomobject]@{ Name = "Ic"; Property = "Ic"; Color = [System.Drawing.Color]::DarkGreen }
    )

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 610 -Width 1270 -Height 170 `
    -Title "Output speed (rpm)" -Range $speedRange -Series @(
        [pscustomobject]@{ Name = "Reference"; Property = "SpeedRef"; Color = [System.Drawing.Color]::SeaGreen },
        [pscustomobject]@{ Name = "Feedback"; Property = "SpeedFdbk"; Color = [System.Drawing.Color]::RoyalBlue }
    )

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 875 -Width 1270 -Height 140 `
    -Title "Inner shaft speed (rpm)" -Range $innerSpeedRange -Series @(
        [pscustomobject]@{ Name = "Reference"; Property = "InnerSpeedRef"; Color = [System.Drawing.Color]::SeaGreen },
        [pscustomobject]@{ Name = "Feedback"; Property = "InnerSpeedFdbk"; Color = [System.Drawing.Color]::RoyalBlue }
    )

$last = $rows[$rows.Count - 1]
$summary = ("Last: out={0:F3} rpm  inner={1:F3} rpm  CCR={2:F0}/{3:F0}/{4:F0}  Ia={5:F3} A  Ib={6:F3} A  Ic={7:F3} A" -f
    $last.SpeedFdbk, $last.InnerSpeedFdbk, $last.CcrA, $last.CcrB, $last.CcrC,
    $last.Ia, $last.Ib, $last.Ic)
$graphics.DrawString($summary, $labelFont, $textBrush, 90, 1025)
$graphics.DrawString("CSV contains all 10 channels.", $smallFont, $textBrush, 90, 1048)

$bitmap.Save($pngPath, [System.Drawing.Imaging.ImageFormat]::Png)
$bitmap.Save($latestPngPath, [System.Drawing.Imaging.ImageFormat]::Png)

$graphics.Dispose()
$bitmap.Dispose()
$titleFont.Dispose()
$panelFont.Dispose()
$labelFont.Dispose()
$smallFont.Dispose()
$gridPen.Dispose()
$axisPen.Dispose()

Write-Output "CSV=$csvPath"
Write-Output "PNG=$pngPath"
Write-Output "LATEST_PNG=$latestPngPath"
Write-Output $summary
