# mota_push.ps1 — expansion module firmware push: cloud -> 757 littlefs repo (by model t<type>.fw) -> mota-flash backplane streaming flash
# usage: .\mota_push.ps1 -Public [-Type 1] [-Addr 2] [-Elf <h503 elf>] [-NoFlash]
# contract=Backplane_Bus_Protocol.md v0.24; model registry=docs/Board_Type_and_Version_Registry.md; during streaming flash the master self-checks the slave model matches.
param(
    [string]$Elf = "$PSScriptRoot\..\platform_h503\build\Debug\platform_h503.elf",
    [int]$Type = 1,     # board model code (1=EX_16O, 2=PH_EC; number assigned in registry)
    [int]$Addr = 2,
    [int]$GapMs = 60,
    [string]$Sn = 'YOUR_BOARD_SN',   # target main-controller serial number (= device certificate CN), topics = dev/I757-M/<Sn>/*
    [switch]$Public,
    [switch]$NoFlash    # only store in repo, no streaming flash (repo pre-warm)
)
$T_CMD = "dev/I757-M/$Sn/dn/cmd"; $T_FW = "dev/I757-M/$Sn/dn/fw"
$BrokerIp = '127.0.0.1'; $BrokerPort = 1883; $User=''; $Pass=''; $Tls=$false
$KeysDir = "$PSScriptRoot\..\..\keys\ca"
if ($Public) { $BrokerIp='YOUR_BROKER_HOST'; $BrokerPort=18884; $Tls=$true }   # <<< set to your broker
$mos = 'D:\Program Files\Mosquitto'
$objcopy = 'D:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-objcopy.exe'
$mosAuth = @('-p', $BrokerPort)
if ($Tls) { $mosAuth += @('--cafile', "$KeysDir\ca.crt", '--cert', "$KeysDir\device-0001.crt", '--key', "$KeysDir\device-0001.key") }

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
function Pub([string]$topic, [string]$msg) { & "$mos\mosquitto_pub.exe" -h $BrokerIp @mosAuth -t $topic -m $msg }
function Publish-BinaryChunks([byte[][]]$payloads, [string]$topic, [int]$gapMs) {
    $tc = New-Object Net.Sockets.TcpClient($BrokerIp, $BrokerPort)
    if ($Tls) {
        $cert = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2("$KeysDir\device-0001.pfx", 'i757')
        $certs = New-Object System.Security.Cryptography.X509Certificates.X509Certificate2Collection($cert)
        $cb = [System.Net.Security.RemoteCertificateValidationCallback]{ param($s, $c, $ch, $e) $true }
        $ns = New-Object System.Net.Security.SslStream($tc.GetStream(), $false, $cb)
        $ns.AuthenticateAsClient($BrokerIp, $certs, [System.Security.Authentication.SslProtocols]::Tls12, $false)
    } else { $ns = $tc.GetStream() }
    function EncLen([int]$n) { $out = @(); do { $b = $n % 128; $n = [int][math]::Floor($n / 128); if ($n -gt 0) { $b = $b -bor 0x80 }; $out += [byte]$b } while ($n -gt 0); return [byte[]]$out }
    $cid = [Text.Encoding]::ASCII.GetBytes("mota-pusher")
    $vh = [byte[]](0,4) + [Text.Encoding]::ASCII.GetBytes("MQTT") + [byte[]](4, 2, 0, 60) + [byte[]](0, $cid.Length) + $cid
    $pkt = [byte[]](0x10) + (EncLen $vh.Length) + $vh
    $ns.Write($pkt, 0, $pkt.Length)
    $ack = New-Object byte[] 4; $null = $ns.Read($ack, 0, 4)
    if ($ack[3] -ne 0) { throw "MQTT CONNACK refused: $($ack[3])" }
    $t = [Text.Encoding]::ASCII.GetBytes($topic)
    $i = 0
    foreach ($pl in $payloads) {
        $body = [byte[]](0, $t.Length) + $t + $pl
        $pkt = [byte[]](0x30) + (EncLen $body.Length) + $body
        $ns.Write($pkt, 0, $pkt.Length); $ns.Flush()
        Start-Sleep -Milliseconds $gapMs
        $i++
        if (($i % 10) -eq 0) { Write-Host (">>   chunk {0}/{1}" -f $i, $payloads.Count) }
    }
    $ns.Write([byte[]](0xE0, 0), 0, 2)
    $tc.Close()
}

$bin = [IO.Path]::Combine($env:TEMP, "mota_t$Type.bin")
& $objcopy -O binary $Elf $bin
$bytes = [IO.File]::ReadAllBytes($bin)
$crc = Get-Crc32 $bytes
$crcHex = '{0:x8}' -f $crc
Write-Host (">> module fw type={0}: {1} bytes crc={2}" -f $Type, $bytes.Length, $crcHex)
Pub $T_CMD ("mota-begin {0} {1} {2}" -f $Type, $bytes.Length, $crcHex)
Start-Sleep -Milliseconds 800

$chunks = @()
for ($o = 0; $o -lt $bytes.Length; $o += 1024) {
    $n = [Math]::Min(1024, $bytes.Length - $o)
    $chunks += , ([byte[]]($bytes[$o..($o + $n - 1)]))
}
Write-Host (">> pushing {0} chunks to repo (littlefs t{1}.fw)..." -f $chunks.Count, $Type)
Publish-BinaryChunks $chunks $T_FW $GapMs
Start-Sleep -Seconds 2

if (-not $NoFlash) {
    Write-Host ">> mota-flash $Addr (watch [MOTA] on COM8; slave swaps + 3-strike guard)"
    Pub $T_CMD ("mota-flash {0}" -f $Addr)
}
Write-Host ">> done (verify: slave ident fw_ver via console 'mbus'/'mbx' or COM8 [MOTA] log)"
