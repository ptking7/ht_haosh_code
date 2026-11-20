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

$tail = [byte[]](0x00, 0x00, 0x80, 0x7f)
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
                        $buffer.RemoveRange(0, $buffer.Count - 67)
                    }
                    break
                }

                if ($tailIndex -ge 64) {
                    $values = New-Object "float[]" 16
                    for ($ch = 0; $ch -lt 16; $ch++) {
                        $valueBytes = $buffer.GetRange(
                            $tailIndex - 64 + $ch * 4, 4).ToArray()
                        $values[$ch] = [BitConverter]::ToSingle($valueBytes, 0)
                    }

                    $rows.Add([pscustomobject]@{
                        Time       = [Math]::Round($stopwatch.Elapsed.TotalSeconds, 4)
                        Id         = $values[0]
                        Iq         = $values[1]
                        IdRef      = $values[2]
                        IqRef      = $values[3]
                        Ud         = $values[4]
                        Uq         = $values[5]
                        Imag       = $values[6]
                        Angle      = $values[7]
                        CalibFlag  = $values[8]
                        State      = $values[9]
                        Error      = $values[10]
                        IdRaw      = $values[11]
                        IqRaw      = $values[12]
                        SpeedRef   = $values[13]
                        SpeedFdbk  = $values[14]
                        Bus        = $values[15]
                    })
                }

                $buffer.RemoveRange(0, $tailIndex + 4)
            }
        }
        else {
            Start-Sleep -Milliseconds 5
        }

        $elapsed = [Math]::Floor($stopwatch.Elapsed.TotalSeconds)
        if (($elapsed -ge $lastProgress + 5) -and ($rows.Count -gt 0)) {
            $last = $rows[$rows.Count - 1]
            Write-Output ("t={0,2}s frames={1,5} Id={2,7:F3} Iq={3,7:F3} flag={4:N0} err={5:N0}" -f
                $elapsed, $rows.Count, $last.Id, $last.Iq, $last.CalibFlag, $last.Error)
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
    throw "No JustFloat frames were received from $Port."
}

$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$csvPath = Join-Path $outputPath "current_loop_$stamp.csv"
$pngPath = Join-Path $outputPath "current_loop_$stamp.png"
$latestPngPath = Join-Path $outputPath "current_loop_latest.png"
$rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8

$first5 = -1
$first6 = -1
for ($i = 0; $i -lt $rows.Count; $i++) {
    if (($first5 -lt 0) -and ($rows[$i].CalibFlag -ge 5)) {
        $first5 = $i
    }
    if (($first6 -lt 0) -and ($rows[$i].CalibFlag -ge 6)) {
        $first6 = $i
        break
    }
}

if ($first5 -ge 0) {
    $focusStart = [Math]::Max(0, $first5 - 100)
    if ($first6 -ge 0) {
        $focusEnd = [Math]::Min($rows.Count - 1, $first6 + 300)
    }
    else {
        $focusEnd = [Math]::Min($rows.Count - 1, $first5 + 220)
    }
    $plotRows = @($rows.GetRange($focusStart, $focusEnd - $focusStart + 1))
}
else {
    $plotRows = @($rows)
}

$imageWidth = 1400
$imageHeight = 900
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
    "Current-loop capture  $Port  $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    $titleFont, $textBrush, 35, 16)

function Get-Range {
    param($Data, $Properties)
    $values = New-Object "System.Collections.Generic.List[double]"
    foreach ($row in $Data) {
        foreach ($property in $Properties) {
            $value = [double]$row.$property
            if (-not [double]::IsNaN($value) -and
                -not [double]::IsInfinity($value)) {
                $values.Add($value)
            }
        }
    }
    if ($values.Count -eq 0) {
        return @(-1.0, 1.0)
    }
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
    param(
        $G,
        $Data,
        [int]$Left,
        [int]$Top,
        [int]$Width,
        [int]$Height,
        [string]$Title,
        $Series,
        $Range
    )

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
            if ([double]::IsNaN($value) -or [double]::IsInfinity($value)) {
                $lastPoint = $null
                continue
            }
            if ($value -lt $yMin) {
                $value = $yMin
            }
            if ($value -gt $yMax) {
                $value = $yMax
            }
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

$currentRange = Get-Range $plotRows @("Id", "Iq", "IdRef", "IqRef")
$voltageRange = Get-Range $plotRows @("Ud", "Uq")
$speedRange = Get-Range $plotRows @("SpeedRef", "SpeedFdbk")

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 80 -Width 1270 -Height 210 `
    -Title "Current (A)" -Range $currentRange -Series @(
        [pscustomobject]@{ Name = "Id"; Property = "Id"; Color = [System.Drawing.Color]::RoyalBlue },
        [pscustomobject]@{ Name = "Iq"; Property = "Iq"; Color = [System.Drawing.Color]::OrangeRed },
        [pscustomobject]@{ Name = "IdRef"; Property = "IdRef"; Color = [System.Drawing.Color]::DarkGreen },
        [pscustomobject]@{ Name = "IqRef"; Property = "IqRef"; Color = [System.Drawing.Color]::Purple }
    )

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 345 -Width 1270 -Height 210 `
    -Title "Voltage (V)" -Range $voltageRange -Series @(
        [pscustomobject]@{ Name = "Ud"; Property = "Ud"; Color = [System.Drawing.Color]::RoyalBlue },
        [pscustomobject]@{ Name = "Uq"; Property = "Uq"; Color = [System.Drawing.Color]::OrangeRed }
    )

Draw-Panel -G $graphics -Data $plotRows -Left 90 -Top 610 -Width 1270 -Height 170 `
    -Title "External speed (rpm)" -Range $speedRange -Series @(
        [pscustomobject]@{ Name = "SpeedRef"; Property = "SpeedRef"; Color = [System.Drawing.Color]::SeaGreen },
        [pscustomobject]@{ Name = "SpeedFdbk"; Property = "SpeedFdbk"; Color = [System.Drawing.Color]::RoyalBlue }
    )

$last = $rows[$rows.Count - 1]
$summary = ("Last: Id={0:F3} A  Iq={1:F3} A  IdRef={2:F3} A  Ud={3:F3} V  Uq={4:F3} V  flag={5:N0}  error={6:N0}" -f
    $last.Id, $last.Iq, $last.IdRef, $last.Ud, $last.Uq, $last.CalibFlag, $last.Error)
$graphics.DrawString($summary, $labelFont, $textBrush, 90, 820)
$graphics.DrawString(
    "Plot window: calib_flag >= 5; full data in CSV.",
    $smallFont, $textBrush, 90, 845)

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
