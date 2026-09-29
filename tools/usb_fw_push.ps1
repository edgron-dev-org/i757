# usb_fw_push.ps1 — push firmware over the USB-C console (OTA for sites with no network; board side = CM7/App/app_usb_fw.c)
#
# Module mode (default):  .\usb_fw_push.ps1 -Port COM8 [-Type 1] [-Addr 2] -Elf <module .elf>
#   = the USB flavour of mota_push: mota-begin into the on-board repo -> fwchunk stream -> mota-flash over the backplane
# Self mode:              .\usb_fw_push.ps1 -Port COM8 -Self -Cm7Elf <CM7.elf> -Cm4Elf <CM4.elf> -SignKey <your .key> [-NoApply]
#   = the USB flavour of ota_push: two segments ota-begin / OTA1 chunk stream -> ota-sign (private key stays on this PC)
#     -> ota-apply -> board resets into the other bank (the COM port disappears and re-enumerates) -> read the bank back
#     -> `ota-confirm yes` (on-site promotion: with no cloud to confirm against, the trial period would roll back)
# Protocol: console text commands pass through verbatim to the board's ota_cmd; "fwchunk <n>" followed by n raw bytes is
#           the equivalent of one dn/fw MQTT message; the board answers on the console ("go" / "ok" / "[MOTA]..." / "[OTA]...").
# Never run this concurrently with a cloud OTA (the engine is single-session). If the transfer is interrupted just rerun
# the whole script (chunks are offset-idempotent, mota-begin resets the repo).
# The signing key must be one the board accepts: the customer key registered with `fwkey2 set`
# (OTA_and_MQTT_User_Guide.md 6.2bis) or the Edgron release key.
param(
    [string]$Port = 'COM8',
    [switch]$Self,
    # module mode
    [int]$Type = 1,          # module type code (1 = EX-16DO, 2 = PH_EC, 3 = EX-16DI; Board_Type registry)
    [int]$Addr = 2,          # backplane address of the module to flash (DIP + 1)
    [string]$Elf = "",       # module firmware .elf
    # self mode
    [string]$Cm7Elf = "$PSScriptRoot\..\platform_757\CM7\build\Debug\platform_757_CM7.elf",
    [string]$Cm4Elf = "$PSScriptRoot\..\platform_757\CM4\build\Debug\platform_757_CM4.elf",
    [switch]$NoApply,
    [string]$SignKey = "",   # firmware-signing private key (PEM, EC P-256); default keys/fwsign/fwsign.key if that exists
    [string]$Objcopy = "arm-none-eabi-objcopy",   # from STM32CubeCLT / GNU Arm toolchain on PATH, or a full path
    [string]$Openssl = ""    # default: Git for Windows' openssl.exe, else `openssl` on PATH
)
$CHUNK = 1024        # matches the board's de-duplication bitmap granularity (OTA_CHUNK_SHIFT); module path has no such constraint
$CM4_OFFSET = 0x80000

function Get-Crc32([byte[]]$bytes) {
    $table = New-Object long[] 256
    for ($i = 0; $i -lt 256; $i++) {
        [long]$c = $i
        for ($k = 0; $k -lt 8; $k++) { if ($c -band 1L) { $c = 0xEDB88320L -bxor ($c -shr 1) } else { $c = $c -shr 1 } }
        $table[$i] = $c
    }
    [long]$crc = 0xFFFFFFFFL
    foreach ($b in $bytes) { $crc = $table[($crc -bxor $b) -band 0xFFL] -bxor ($crc -shr 8) }
    return (($crc -bxor 0xFFFFFFFFL) -band 0xFFFFFFFFL)
}

$script:sp = $null
function Open-Port([int]$timeoutS = 30) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        try {
            $script:sp = New-Object System.IO.Ports.SerialPort($Port, 115200)
            $script:sp.ReadTimeout = 200; $script:sp.WriteTimeout = 3000
            $script:sp.Open(); $script:sp.DiscardInBuffer(); return
        } catch { Start-Sleep -Milliseconds 800 }
    }
    throw "cannot open $Port (${timeoutS}s)"
}
function Send-Line([string]$s) { $script:sp.Write("$s`r") }
function Read-Until([string]$pattern, [int]$timeoutS, [switch]$Quiet) {
    # collect console output until it matches $pattern; echo everything to the operator (the board's [MOTA]/[OTA] log is the progress bar)
    $buf = ''; $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        try { $s = $script:sp.ReadExisting() } catch { $s = '' }
        if ($s) {
            if (-not $Quiet) { Write-Host -NoNewline $s }
            $buf += $s
            if ($buf -cmatch $pattern) { return $buf }   # case-sensitive: the heartbeat line "RPC-OK" must not match a lowercase ok
        } else { Start-Sleep -Milliseconds 50 }
    }
    throw "timeout waiting for '$pattern' (${timeoutS}s)"
}
function Send-Chunk([byte[]]$payload) {
    Send-Line ("fwchunk {0}" -f $payload.Length)
    $null = Read-Until 'go' 10 -Quiet
    $script:sp.Write($payload, 0, $payload.Length)
    # Reply matching, three traps seen in the field: "crc-ok" contains ok, the heartbeat line "ierr=0" contains err, and the
    # prompt "i757> " has no newline so the reply arrives as "i757> ok". Negative look-behinds accept whole words only.
    $r = Read-Until '((?<![-\w])ok(?=[\r\n]))|((?<![\w])err[^\r\n]*)' 15 -Quiet
    if ($r -cmatch '(?<![\w])err') { throw "fwchunk rejected: $r" }
    return $r
}
function Poll-OtaState([string]$state, [int]$timeoutS) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        Send-Line 'ota'
        $r = Read-Until 'state=\S+' 8 -Quiet
        if ($r -match "state=$state") { return }
        Start-Sleep -Seconds 2
    }
    throw "timeout waiting for ota state=$state (${timeoutS}s)"
}
function To-Bin([string]$elf) {
    $bin = [IO.Path]::ChangeExtension($elf, '.usb.bin')
    & $Objcopy -O binary $elf $bin
    if ($LASTEXITCODE -ne 0) { throw "objcopy failed ($Objcopy): is the GNU Arm toolchain on PATH? (-Objcopy <full path>)" }
    return $bin
}

Open-Port 10
Send-Line ''            # wake the prompt, and confirm a live CLI is on the other end
$null = Read-Until 'i757>' 5 -Quiet

if (-not $Self) {
    # ================= module mode (backplane slave such as the H503-based modules) =================
    if (-not $Elf) { throw 'module mode needs -Elf <module firmware .elf> (or use -Self for the controller itself)' }
    $Elf = (Resolve-Path $Elf).Path
    $bytes = [IO.File]::ReadAllBytes((To-Bin $Elf))
    $crc = Get-Crc32 $bytes
    Write-Host (">> module firmware type={0}: {1} bytes crc={2:x8}" -f $Type, $bytes.Length, $crc)
    Send-Line ("mota-begin {0} {1} {2:x8}" -f $Type, $bytes.Length, $crc)
    $null = Read-Until '\[MOTA\] recv' 10
    $n = [math]::Ceiling($bytes.Length / $CHUNK)
    for ($i = 0; $i -lt $n; $i++) {
        $off = $i * $CHUNK
        $take = [math]::Min($CHUNK, $bytes.Length - $off)
        $pl = New-Object byte[] $take
        [Array]::Copy($bytes, $off, $pl, 0, $take)
        $ack = Send-Chunk $pl
        if ((($i + 1) % 10) -eq 0) { Write-Host (">>   chunk {0}/{1}" -f ($i + 1), $n) }
    }
    # the "[MOTA] stored ... crc-ok" line usually arrives with the last chunk's reply (already in $ack); otherwise wait for it
    if ($ack -notmatch 'crc-ok') { $null = Read-Until 'crc-ok' 8 }
    Write-Host "`n>> stored in the on-board repo, streaming to module $Addr ..."
    Send-Line ("mota-flash {0}" -f $Addr)
    $r = Read-Until '(OTA OK|FAIL|ident failed|not in repo|timeout)' 90
    if ($r -notmatch 'OTA OK') { throw 'module flash did not succeed, see the [MOTA] log above' }
    Write-Host "`n>> USB module OTA complete" -ForegroundColor Green
}
else {
    # ================= self mode (the controller's two cores) =================
    $Cm7Elf = (Resolve-Path $Cm7Elf).Path; $Cm4Elf = (Resolve-Path $Cm4Elf).Path
    $parts = @( @{ name='CM7'; elf=$Cm7Elf; offset=0 },
                @{ name='CM4'; elf=$Cm4Elf; offset=$CM4_OFFSET } )
    foreach ($p in $parts) {
        $bytes = [IO.File]::ReadAllBytes((To-Bin $p.elf))
        $crc = Get-Crc32 $bytes
        Copy-Item $p.elf ([IO.Path]::ChangeExtension($p.elf, (".pushed_{0:x8}.elf" -f $crc))) -Force   # keep the ELF that went out, for later debugging
        Write-Host (">> {0}: {1} bytes crc={2:x8}" -f $p.name, $bytes.Length, $crc)
        Send-Line ("ota-begin {0:x} {1} {2:x8}" -f $p.offset, $bytes.Length, $crc)
        Poll-OtaState 'ready' 120           # bank erase
        $n = [math]::Ceiling($bytes.Length / $CHUNK)
        for ($i = 0; $i -lt $n; $i++) {
            $off = $i * $CHUNK
            $take = [math]::Min($CHUNK, $bytes.Length - $off)
            $msg = New-Object byte[] (8 + $take)
            [Text.Encoding]::ASCII.GetBytes('OTA1').CopyTo($msg, 0)
            [BitConverter]::GetBytes([uint32]$off).CopyTo($msg, 4)
            [Array]::Copy($bytes, $off, $msg, 8, $take)
            $null = Send-Chunk $msg
            if ((($i + 1) % 25) -eq 0) { Write-Host (">>   {0} chunk {1}/{2}" -f $p.name, ($i + 1), $n) }
        }
        Poll-OtaState 'verify-ok' 30
        Write-Host (">> {0}: verify-ok" -f $p.name)
    }
    # signature, same form as ota_push / fw_sign: ECDSA-P256 over cm7 || cm4; the private key never leaves this PC
    if (-not $Openssl) {
        $Openssl = 'C:\Program Files\Git\usr\bin\openssl.exe'
        if (-not (Test-Path $Openssl)) { $Openssl = 'openssl' }
    }
    $FwKey = if ($SignKey) { (Resolve-Path $SignKey).Path } else { "$PSScriptRoot\..\..\keys\fwsign\fwsign.key" }
    if (-not (Test-Path $FwKey)) { throw "signing key not found: $FwKey (pass -SignKey <your .key>, registered on the board with 'fwkey2 set')" }
    $cm7bin = [IO.File]::ReadAllBytes([IO.Path]::ChangeExtension($Cm7Elf, '.usb.bin'))
    $cm4bin = [IO.File]::ReadAllBytes([IO.Path]::ChangeExtension($Cm4Elf, '.usb.bin'))
    $combined = New-Object byte[] ($cm7bin.Length + $cm4bin.Length)
    [Array]::Copy($cm7bin, 0, $combined, 0, $cm7bin.Length)
    [Array]::Copy($cm4bin, 0, $combined, $cm7bin.Length, $cm4bin.Length)
    $tmpImg = [IO.Path]::Combine($env:TEMP, 'i757_usb_fw.bin'); $tmpSig = [IO.Path]::Combine($env:TEMP, 'i757_usb_fw.sig')
    [IO.File]::WriteAllBytes($tmpImg, $combined)
    & $Openssl dgst -sha256 -sign $FwKey -out $tmpSig $tmpImg
    if ($LASTEXITCODE -ne 0) { throw 'openssl signing failed' }
    $sigHex = ([IO.File]::ReadAllBytes($tmpSig) | ForEach-Object { $_.ToString('x2') }) -join ''
    Remove-Item $tmpImg, $tmpSig -ErrorAction SilentlyContinue
    Send-Line "ota-sign $sigHex"
    Poll-OtaState 'signed' 20
    Write-Host '>> signature verified on the board (footer written)'
    if ($NoApply) { Write-Host '>> -NoApply: stopping here; run ota-apply on the console when ready'; $script:sp.Close(); exit 0 }

    Send-Line 'ota'
    $bankBefore = ''
    if ((Read-Until 'bank=\d' 8 -Quiet) -match 'bank=(\d)') { $bankBefore = $Matches[1] }
    Write-Host ">> apply: bank swap + reset (the USB port drops and re-enumerates)..."
    Send-Line 'ota-apply'
    Start-Sleep -Seconds 3
    try { $script:sp.Close() } catch {}
    Start-Sleep -Seconds 12
    Open-Port 60
    Send-Line ''; $null = Read-Until 'i757>' 10 -Quiet
    Send-Line 'ota'
    $r = Read-Until 'bank=\d' 10
    if ($bankBefore -and ($r -match 'bank=(\d)') -and ($Matches[1] -eq $bankBefore)) {
        # option-byte bank swap refused inside the early-boot window: wait it out and send apply once more
        Write-Host ">> bank did not swap (still $bankBefore), waiting 60 s for the early-boot window to close, then re-applying..."
        Start-Sleep -Seconds 60
        Send-Line 'ota-apply'
        Start-Sleep -Seconds 3
        try { $script:sp.Close() } catch {}
        Start-Sleep -Seconds 12
        Open-Port 60
        Send-Line ''; $null = Read-Until 'i757>' 10 -Quiet
        Send-Line 'ota'
        $r = Read-Until 'bank=\d' 10
        if (($r -match 'bank=(\d)') -and ($Matches[1] -eq $bankBefore)) { throw 'bank swap did not take effect after two attempts' }
    }
    Write-Host "`n>> new firmware running (trial period), promoting on site..."
    Send-Line 'ota-confirm yes'
    $null = Read-Until '(CONFIRMED|not in trial)' 30
    Write-Host "`n>> USB self OTA complete" -ForegroundColor Green
}
try { $script:sp.Close() } catch {}
