# ota_push.ps1 — push dual-core firmware to the board and trigger bank swap
# usage: .\ota_push.ps1 [-Apply] [-RevertOnly] [-Public] [-Sn <board sn>]
#   -Public = use your public broker (mTLS, in sync with app_cfg.h/vps_mqtt_setup.sh)
#   -Sn     = target board serial number (= device certificate CN); topics = dev/I757-M/<Sn>/* (Device Cloud Protocol §3), each board gets its own push
#   -LegacyNs = pre-2026-07-24 global i757/* topics (only for the final push migrating a board still on old firmware; remove afterwards)
#   -SignKey / -ClientCert = sign and push with YOUR OWN key (2026-09-07): your signing private key + the cust-<company>
#             broker certificate Edgron issued to you; the board must already carry your public key (`fwkey2 set`, User Guide 6.2bis)
param(
    [string]$Cm7Elf = "$PSScriptRoot\..\platform_nucleo\CM7\build\platform_nucleo_CM7.elf",
    [string]$Cm4Elf = "$PSScriptRoot\..\platform_nucleo\CM4\build\platform_nucleo_CM4.elf",
    [string]$BrokerIp = "192.168.7.1",
    [int]$BrokerPort = 1883,
    [string]$User = "",
    [string]$Pass = "",
    [string]$Sn = "YOUR_BOARD_SN",
    [string]$SignKey = "",          # firmware-signing private key (PEM, EC P-256); default = keys/fwsign/fwsign.key. Customers: your own key registered on the board with `fwkey2 set` (User Guide 6.2bis)
    [string]$ClientCert = "device-0001",   # broker client identity for -Public: <name>.crt/.key/.pfx in keys/ca (or a path prefix). Customers: cust-<company> as issued by Edgron
    [string]$PfxPass = "i757",             # password of <ClientCert>.pfx
    [switch]$LegacyNs,
    [int]$GapMs = 30,
    [switch]$Public,
    [switch]$Aws,        # use AWS IoT Core (Sydney 8883, device certificate mTLS, trust anchor=Amazon Root CA 1)
    [switch]$Apply = $true,
    [switch]$RevertOnly,
    [switch]$NoPostVerify,   # after swap do not wait for heartbeat on push broker (after AWS push the board reboots back to VPS, verified by caller)
    [switch]$TamperSig   # security test: sign an image with 1 byte changed -> DER valid but signature mismatches -> board should sig-fail and refuse the swap
)
# Resolve ELF paths to absolute right away: .NET file APIs ([IO.File]::ReadAllBytes etc.) do not
# follow PowerShell's Set-Location, so a relative path resolves against the process CWD and
# objcopy silently produces a 0-byte image (measured: the board receives "ota-begin 0 0").
$Cm7Elf = (Resolve-Path $Cm7Elf).Path
$Cm4Elf = (Resolve-Path $Cm4Elf).Path
# per-device topics (Device Cloud Protocol §3); -LegacyNs = pre-migration global topics
if ($LegacyNs) { $T_STAT = 'i757/status'; $T_CMD = 'i757/dn/cmd'; $T_FW = 'i757/dn/fw' }
else { $T_STAT = "dev/I757-M/$Sn/up/status"; $T_CMD = "dev/I757-M/$Sn/dn/cmd"; $T_FW = "dev/I757-M/$Sn/dn/fw" }
$KeysDir = "$PSScriptRoot\..\..\keys\ca"
$CaFile = "$KeysDir\ca.crt"   # default our CA (used for VPS); switched to Amazon Root CA for AWS
$Tls = $false
if ($Public) {
    # mTLS port: device certificate = identity, no username/password.
    # >>> Set to YOUR broker host <<<
    $BrokerIp = 'YOUR_BROKER_HOST'; $BrokerPort = 18884
    $User = ''; $Pass = ''; $Tls = $true
    if (-not $PSBoundParameters.ContainsKey('GapMs')) { $GapMs = 100 }
}
if ($Aws) {
    # AWS IoT Core mTLS: same device cert, trust anchor = Amazon Root CA 1; domain access (SslStream adds SNI).
    # >>> Set to YOUR AWS IoT endpoint <<<
    $BrokerIp = 'YOUR_AWS_IOT_ENDPOINT'; $BrokerPort = 8883
    $User = ''; $Pass = ''; $Tls = $true
    $CaFile = "$PSScriptRoot\..\..\keys\aws\AmazonRootCA1.pem"
    if (-not $PSBoundParameters.ContainsKey('GapMs')) { $GapMs = 120 }
}
Write-Host ">> broker=${BrokerIp}:${BrokerPort} auth=$([bool]$User) gap=${GapMs}ms"
if ($env:OTA_DRYRUN) { exit 0 }
$ErrorActionPreference = 'Continue'  # mosquitto_sub timeout writes to stderr, under PS5.1 a Stop policy would wrongly kill a normal wait
$mos = 'D:\Program Files\Mosquitto'
$objcopy = 'D:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe'
$CM4_OFFSET = 0x80000
$mosAuth = @('-p', $BrokerPort)
# client identity: a bare name lives in keys/ca; anything with a path separator is used as a prefix as-is
$ClientPfx = if ($ClientCert -match '[\\/]') { $ClientCert } else { "$KeysDir\$ClientCert" }
if ($Tls)       { $mosAuth += @('--cafile', $CaFile, '--cert', "$ClientPfx.crt", '--key', "$ClientPfx.key") }
elseif ($User)  { $mosAuth += @('-u', $User, '-P', $Pass) }
if ($Aws)       { $mosAuth += @('-i', 'ota-pusher-sub') }   # AWS requires client-id, and different from the pusher to avoid mutual kick-off

function Get-Crc32([byte[]]$bytes) {
    # PS5.1 has no uint32 literal, use long arithmetic throughout
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
function Pub([string]$topic, [string]$msg) { & "$mos\mosquitto_pub.exe" -h $BrokerIp @mosAuth -t $topic -m $msg }

# native MQTT binary publish (mosquitto_pub -f on Windows reads the file in text mode, truncates at 0x1A -> real firmware will surely break)
function Publish-BinaryChunks([byte[][]]$payloads, [string]$topic, [int]$gapMs) {
    $tc = New-Object Net.Sockets.TcpClient($BrokerIp, $BrokerPort)
    if ($Tls) {
        # mTLS: device certificate (PFX) as client identity; server verification simplified to accept (self-built CA private chain, mosquitto tool path already does full verification)
        $cert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2("$ClientPfx.pfx", $PfxPass)
        $certs = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2Collection($cert)
        $cb = [System.Net.Security.RemoteCertificateValidationCallback]{ param($s, $c, $ch, $e) $true }
        $ns = New-Object System.Net.Security.SslStream($tc.GetStream(), $false, $cb)
        $ns.AuthenticateAsClient($BrokerIp, $certs, [System.Security.Authentication.SslProtocols]::Tls12, $false)
    } else {
        $ns = $tc.GetStream()
    }
    function EncLen([int]$n) { $out = @(); do { $b = $n % 128; $n = [int][math]::Floor($n / 128); if ($n -gt 0) { $b = $b -bor 0x80 }; $out += [byte]$b } while ($n -gt 0); return [byte[]]$out }
    $cid = [Text.Encoding]::ASCII.GetBytes("ota-pusher")
    $flags = 2; $tail = [byte[]]@()   # CONNECT flags: clean session (+0x80 user +0x40 pass)
    if ($User) {
        $ub = [Text.Encoding]::ASCII.GetBytes($User); $pb = [Text.Encoding]::ASCII.GetBytes($Pass)
        $flags = $flags -bor 0xC0
        $tail = [byte[]](0, $ub.Length) + $ub + [byte[]](0, $pb.Length) + $pb
    }
    $vh = [byte[]](0,4) + [Text.Encoding]::ASCII.GetBytes("MQTT") + [byte[]](4, $flags, 0, 60) + [byte[]](0, $cid.Length) + $cid + $tail
    $pkt = [byte[]](0x10) + (EncLen $vh.Length) + $vh
    $ns.Write($pkt, 0, $pkt.Length)
    $ack = New-Object byte[] 4; $null = $ns.Read($ack, 0, 4)
    if ($ack[3] -ne 0) { throw "MQTT CONNACK refused: $($ack[3])" }
    $t = [Text.Encoding]::ASCII.GetBytes($topic)
    $i = 0
    foreach ($pl in $payloads) {
        $body = [byte[]](0, $t.Length) + $t + $pl
        $pkt = [byte[]](0x30) + (EncLen $body.Length) + $body
        $ns.Write($pkt, 0, $pkt.Length)
        $ns.Flush()
        Start-Sleep -Milliseconds $gapMs
        $i++
        if (($i % 10) -eq 0) { Write-Host (">>   chunk {0}/{1}" -f $i, $payloads.Count) }
    }
    $ns.Write([byte[]](0xE0, 0), 0, 2)   # DISCONNECT
    $tc.Close()
}
function Wait-OtaState([string]$state, [int]$timeoutS) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        $m = & "$mos\mosquitto_sub.exe" -h $BrokerIp @mosAuth -t $T_STAT -C 1 -W 8 2>$null
        if ($m -match "`"ota`":`"$state`"") { return $m }
    }
    throw "timeout waiting ota=$state (${timeoutS}s)"
}
function Get-Heartbeat([int]$timeoutS = 30) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($sw.Elapsed.TotalSeconds -lt $timeoutS) {
        $m = & "$mos\mosquitto_sub.exe" -h $BrokerIp @mosAuth -t $T_STAT -C 1 -W 8 2>$null
        if ($m) { return $m }
    }
    throw "no heartbeat in ${timeoutS}s"
}

if ($RevertOnly) {
    Write-Host ">> revert: toggle bank swap"
    Pub $T_CMD 'ota-revert'
    Start-Sleep -Seconds 14
    Write-Host (Get-Heartbeat)
    exit 0
}

Write-Host ">> current: $(Get-Heartbeat)"
$parts = @()
$parts += @{ name='CM7'; elf=$Cm7Elf; offset=0 }
$parts += @{ name='CM4'; elf=$Cm4Elf; offset=$CM4_OFFSET }

$CHUNK = 1024   # 1KB: matches the board-side dedup bitmap granularity (OTA_CHUNK_SHIFT), and a single TLS-decrypted pbuf (~1.5K) holds one whole chunk = avoids triggering the mqtt multi-segment parse defect. Also used for the plaintext port (more robust)
foreach ($p in $parts) {
    $bin = [IO.Path]::ChangeExtension($p.elf, ".ota.bin")
    & $objcopy -O binary $p.elf $bin
    $bytes = [IO.File]::ReadAllBytes($bin)
    $crc = Get-Crc32 $bytes
    # archive an ELF for every pushed version: what runs on the board may not be the latest build, frozen forensics (addr2line) needs the matching symbol table
    Copy-Item $p.elf ([IO.Path]::ChangeExtension($p.elf, (".pushed_{0:x8}.elf" -f $crc))) -Force
    $cmd = "ota-begin {0:x} {1} {2:x8}" -f $p.offset, $bytes.Length, $crc
    Write-Host (">> {0}: {1} bytes crc={2:x8} -> {3}" -f $p.name, $bytes.Length, $crc, $cmd)
    Pub $T_CMD $cmd
    Wait-OtaState 'ready' 40 | Out-Null
    $n = [math]::Ceiling($bytes.Length / $CHUNK)
    $chunks = @()
    for ($i = 0; $i -lt $n; $i++) {
        $off = $i * $CHUNK
        $take = [math]::Min($CHUNK, $bytes.Length - $off)
        $msg = New-Object byte[] (8 + $take)
        [Text.Encoding]::ASCII.GetBytes('OTA1').CopyTo($msg, 0)
        [BitConverter]::GetBytes([uint32]$off).CopyTo($msg, 4)
        [Array]::Copy($bytes, $off, $msg, 8, $take)
        $chunks += , $msg
    }
    $ok = $false
    for ($round = 1; $round -le 3 -and -not $ok; $round++) {   # chunks are idempotent by offset, just re-push a whole round on lost chunks
        Write-Host ">> $($p.name): pushing $n chunks (round $round, gap ${GapMs}ms)..."
        Publish-BinaryChunks $chunks $T_FW $GapMs
        try { Wait-OtaState 'verify-ok' 25 | Out-Null; $ok = $true } catch { Write-Host ">>   not verified yet, retrying missing chunks" }
    }
    if (-not $ok) { throw "$($p.name): verify failed after 3 rounds" }
    Write-Host ">> $($p.name): verify-ok"
}

# firmware signing (anti-forgery): ECDSA-P256 sign over the two segments cm7||cm4 (same order as board-side SHA256), the board writes the footer / allows swap only if verification passes.
# private key keys/fwsign/fwsign.key stays on this machine only; the board embeds the public key. An attacker without the private key = cannot forge firmware that passes verification.
$openssl = 'C:\Program Files\Git\usr\bin\openssl.exe'
if (-not (Test-Path $openssl)) { $openssl = 'openssl' }
$FwKey = if ($SignKey) { (Resolve-Path $SignKey).Path } else { "$PSScriptRoot\..\..\keys\fwsign\fwsign.key" }
if (-not (Test-Path $FwKey)) { throw "signing key not found: $FwKey (pass -SignKey <your .key>)" }
$cm7bin = [IO.File]::ReadAllBytes([IO.Path]::ChangeExtension($Cm7Elf, ".ota.bin"))
$cm4bin = [IO.File]::ReadAllBytes([IO.Path]::ChangeExtension($Cm4Elf, ".ota.bin"))
$combined = New-Object byte[] ($cm7bin.Length + $cm4bin.Length)
[Array]::Copy($cm7bin, 0, $combined, 0, $cm7bin.Length)
[Array]::Copy($cm4bin, 0, $combined, $cm7bin.Length, $cm4bin.Length)
$tmpImg = [IO.Path]::Combine($env:TEMP, 'i757_fw_combined.bin')
$tmpSig = [IO.Path]::Combine($env:TEMP, 'i757_fw.sig')
if ($TamperSig) { $combined[0] = $combined[0] -bxor 0xFF }   # tamper the signed content -> signature mismatches the image on the board
[IO.File]::WriteAllBytes($tmpImg, $combined)
& $openssl dgst -sha256 -sign $FwKey -out $tmpSig $tmpImg
if ($LASTEXITCODE -ne 0) { throw "openssl signing failed" }
$sigBytes = [IO.File]::ReadAllBytes($tmpSig)
$sigHex = ($sigBytes | ForEach-Object { $_.ToString('x2') }) -join ''
Remove-Item $tmpImg, $tmpSig -ErrorAction SilentlyContinue
Write-Host (">> signing: {0} bytes image, sig {1} bytes -> ota-sign{2}" -f $combined.Length, $sigBytes.Length, $(if ($TamperSig){' [TAMPERED]'}else{''}))
Pub $T_CMD "ota-sign $sigHex"
if ($TamperSig) {
    Start-Sleep -Seconds 4
    Write-Host ">> tamper test, state now: $(Get-Heartbeat 15)"
    Write-Host ">> trying ota-apply (should be REFUSED)..."
    Pub $T_CMD 'ota-apply'
    Start-Sleep -Seconds 8
    Write-Host ">> after apply attempt: $(Get-Heartbeat 20)"
    exit 0
}
Wait-OtaState 'signed' 20 | Out-Null
Write-Host ">> signature verified on device (footer written)"

if ($Apply) {
    Write-Host ">> apply: swapping banks + reset..."
    Pub $T_CMD 'ota-apply'
    if ($NoPostVerify) {
        # after swap the board reboots -> defaults back to VPS (not on the push broker), left for the caller to verify on the VPS
        Write-Host ">> apply sent (post-verify deferred to caller)"
    } else {
        Start-Sleep -Seconds 14
        $hb = Get-Heartbeat 90   # half-open self-heal path: boot ~10s + watchdog 15s + reconnect ≈ 40s, leave margin
        Write-Host ">> after swap: $hb"
    }
}
