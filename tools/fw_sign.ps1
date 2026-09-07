# fw_sign.ps1 — sign a dual-core i757 firmware image (cm7.ota.bin ‖ cm4.ota.bin) with an ECDSA-P256 key.
# Output = the DER signature as hex, i.e. the argument of the board's `ota-sign <hex>` command and
# the "signature" field of the dashboard's firmware-upload page. The private key never leaves this
# machine; the board verifies against its embedded Edgron key OR the customer key registered with
# `fwkey2 set` (OTA_and_MQTT_User_Guide.md §6.2bis).
#
# usage:  .\fw_sign.ps1 -Cm7Bin cm7.ota.bin -Cm4Bin cm4.ota.bin -Key my_fwsign.key [-Out sig.hex]
#         .\fw_sign.ps1 -Cm7Elf ..\platform_757\CM7\build\platform_757_CM7.elf -Cm4Elf ..\platform_757\CM4\build\platform_757_CM4.elf -Key my_fwsign.key
#   ELF inputs are converted with arm-none-eabi-objcopy (-Objcopy to point at it) into <name>.ota.bin next to the ELF.
param(
    [string]$Cm7Bin = "",
    [string]$Cm4Bin = "",
    [string]$Cm7Elf = "",
    [string]$Cm4Elf = "",
    [Parameter(Mandatory = $true)][string]$Key,
    [string]$Out = "",
    [string]$Objcopy = "arm-none-eabi-objcopy",
    [string]$Openssl = ""
)
$ErrorActionPreference = 'Continue'   # openssl talks on stderr; judge by exit codes
if (-not $Openssl) {
    $Openssl = 'C:\Program Files\Git\usr\bin\openssl.exe'
    if (-not (Test-Path $Openssl)) { $Openssl = 'openssl' }
}
function To-Bin([string]$elf, [string]$bin) {
    if ($bin) { return (Resolve-Path $bin).Path }
    if (-not $elf) { throw "give -Cm7Bin/-Cm4Bin or -Cm7Elf/-Cm4Elf" }
    $elf = (Resolve-Path $elf).Path
    $out = [IO.Path]::ChangeExtension($elf, ".ota.bin")
    & $Objcopy -O binary $elf $out
    if ($LASTEXITCODE -ne 0) { throw "objcopy failed ($Objcopy)" }
    return $out
}
$b7 = To-Bin $Cm7Elf $Cm7Bin
$b4 = To-Bin $Cm4Elf $Cm4Bin
$cm7 = [IO.File]::ReadAllBytes($b7)
$cm4 = [IO.File]::ReadAllBytes($b4)
if ($cm7.Length -eq 0 -or $cm4.Length -eq 0) { throw "empty image (cm7=$($cm7.Length) cm4=$($cm4.Length) bytes)" }
# same byte order as the board: SHA256(cm7 ‖ cm4)
$img = New-Object byte[] ($cm7.Length + $cm4.Length)
[Array]::Copy($cm7, 0, $img, 0, $cm7.Length)
[Array]::Copy($cm4, 0, $img, $cm7.Length, $cm4.Length)
$tmpImg = [IO.Path]::Combine($env:TEMP, "i757_fwsign_$PID.bin")
$tmpSig = [IO.Path]::Combine($env:TEMP, "i757_fwsign_$PID.sig")
$tmpPub = [IO.Path]::Combine($env:TEMP, "i757_fwsign_$PID.pub")
try {
    [IO.File]::WriteAllBytes($tmpImg, $img)
    $KeyPath = (Resolve-Path $Key).Path
    & $Openssl dgst -sha256 -sign $KeyPath -out $tmpSig $tmpImg
    if ($LASTEXITCODE -ne 0) { throw "openssl sign failed" }
    # self-check: the signature must verify against this key's own public half before we hand it out
    & $Openssl ec -in $KeyPath -pubout -out $tmpPub 2>$null
    & $Openssl dgst -sha256 -verify $tmpPub -signature $tmpSig $tmpImg | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "self-verify failed (key is not an EC P-256 private key?)" }
    $sig = [IO.File]::ReadAllBytes($tmpSig)
    $hex = ($sig | ForEach-Object { $_.ToString('x2') }) -join ''
    $sha = [BitConverter]::ToString([Security.Cryptography.SHA256]::Create().ComputeHash($img)).Replace('-', '').ToLower()
    Write-Host ("cm7={0} cm4={1} bytes  sha256(cm7||cm4)={2}" -f $cm7.Length, $cm4.Length, $sha)
    Write-Host ("signature: {0} bytes DER" -f $sig.Length)
    Write-Output $hex
    if ($Out) { [IO.File]::WriteAllText($Out, $hex); Write-Host "written: $Out" }
} finally {
    Remove-Item $tmpImg, $tmpSig, $tmpPub -ErrorAction SilentlyContinue
}
