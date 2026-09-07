# Connect Your Boards to Your Own AWS IoT Core

This guide takes you from "no AWS account" to "board heartbeat visible in your own AWS account", end to end: account creation → giving the board an identity **you** own → AWS IoT setup → SDK changes → verification. It assumes you can already build and flash the firmware (see `Software_Manual_Application_Development.md` §2) and that the board has a network connection with internet access.

**How the pieces fit** (the one concept worth internalizing before touching anything):

The secure element (ATECC608) on every board holds **two independent identities in separate key slots**:

| Slot | Identity | Who owns it | Used for |
|---|---|---|---|
| 0 | **Factory identity** — key generated on-chip at production, slot permanently locked; certificate signed by the vendor CA | Vendor | Anti-clone verification, and connections to the vendor-hosted broker |
| `CFG_SE_CUSTOMER_SLOT` (default **2**, see `app_cfg.h`; slots 1–7 are open for you) | **Your identity** — key you generate on-chip with one console command; certificate signed by **your own CA** | **You** | Connections to **your** cloud (this guide) |

Both private keys are generated inside the chip and physically never leave it — you do not handle key files, only certificates (public documents). Your cloud trusts your CA; the vendor never holds your credentials, and you never depend on the vendor's. When the board connects to AWS it automatically presents your identity (once provisioned) and signs the TLS handshake with your slot's key.

## 1. What you need before starting

- A credit/debit card and a phone number (AWS signup).
- The SDK building and flashing successfully (`Software_Manual_Application_Development.md` §2).
- `openssl` on your PC (ships with Git for Windows; native on Linux/macOS).
- A LAN for the board with internet access and a reachable DNS server.
- The board's USB-C console (any serial terminal, 115200).

No certificate files from the vendor are needed — you mint everything yourself below.

AWS IoT Core cost, for scale: the free tier covers 2.25M connection-minutes and 500k messages per month for 12 months; after that, a board publishing one heartbeat every 5 s costs well under USD 1/month. There is no fixed fee.

## 2. Create an AWS account

1. Go to <https://aws.amazon.com> → **Create an AWS Account**. Use a company email; the first identity created is the **root user**.
2. Pick a support plan = **Basic (free)** when asked.
3. Sign in to the AWS Console, and immediately do two hygiene steps:
   - **Enable MFA on the root user** (IAM → root user → assign MFA device).
   - **Create an IAM admin user** for daily work (IAM → Users → Create user → attach the `AdministratorAccess` policy; enable console access). Stop using the root user from here on.
4. **Pick your region** (top-right selector, e.g. `ap-southeast-2` Sydney — choose the one nearest your site) and *stay in it*: certificates, policies, Things and your endpoint are all per-region.

## 3. Install the AWS CLI (recommended)

The console (web UI) works for everything below, but the CLI makes the steps copy-pasteable and repeatable per board. Install from <https://aws.amazon.com/cli/>, then:

```
aws configure
# Access key / secret: IAM → your admin user → Security credentials → Create access key
# Default region: your chosen region, e.g. ap-southeast-2
# Default output: json
```

Console equivalents are noted inline as *(Console: …)*.

## 4. Create your CA and the board's identity (on-board ceremony)

Once per company — **create your CA** (one key, one self-signed cert; guard the `.key` file like a root password):

```
openssl ecparam -name prime256v1 -genkey -noout -out my-ca.key
openssl req -new -x509 -key my-ca.key -subj "/CN=MyCompany Device CA" -days 3650 -out my-ca.crt
```

Then **per board**, on the USB-C console:

1. **Generate the key inside the chip** (destructive for that slot only — the factory identity in slot 0 is untouched and untouchable):
   ```
   id2-gen yes
   id2-gen: E48EC35F...  (128 hex chars = the P-256 public key, X||Y)
   ```
2. **Wrap the public key and sign it with your CA** on the PC. The chip cannot emit a CSR, so the certificate is issued over the raw public key (`-force_pubkey`); the dummy CSR only donates the subject name. **Set CN to the board's serial number** (shown by the `id` command, e.g. `i757-0002`) — the firmware uses the factory serial as the MQTT client id, and the §5 policy pins permissions to it:
   ```
   # paste the 128 hex chars from id2-gen into PUB, then:
   PUB=<128-hex-chars>
   printf '3059301306072a8648ce3d020106082a8648ce3d03010703420004%s' "$PUB" | xxd -r -p > chip-pub.der
   openssl pkey -pubin -inform DER -in chip-pub.der -out chip-pub.pem
   openssl req -new -key my-ca.key -subj "/CN=i757-0002" -out dummy.csr
   openssl x509 -req -in dummy.csr -CA my-ca.crt -CAkey my-ca.key -set_serial 1001 -days 3650 \
                -force_pubkey chip-pub.pem -out dev.crt
   ```
   (On Windows without `xxd`, any hex-to-binary step works — e.g. PowerShell `[Convert]::ToByte()` over the pairs.)
3. **Store the certificate on the board** (single line: `id2-cert ` + the DER as hex):
   ```
   openssl x509 -in dev.crt -outform DER | xxd -p -c 4096      # -> one hex line
   id2-cert 3082016230820107a0...
   id2-cert: OK cn=i757-0002 len=358 (takes effect for TLS on next boot)
   ```
   The board refuses a certificate whose public key does not match the chip slot — a wrong-board mix-up cannot slip through.
4. `id2` shows the stored identity; `id2-cert-show` re-prints it as PEM any time you need the file again (it is public data — losing it is never fatal, and if the file system copy is ever lost, re-run step 3 with the same cert, or re-sign from `id2-pub`).

## 5. Register the certificate with AWS

AWS does not need to know your CA — register the device certificate directly:

```
aws iot register-certificate-without-ca --certificate-pem file://dev.crt --status ACTIVE
```

Note the `certificateArn` in the output. Repeat per board. *(Console: AWS IoT → Security → Certificates → Add certificate → Register certificate → upload → Activate.)*

(At fleet scale you can instead register your CA once and let boards auto-register on first connect — AWS "JITP"; start with the direct path above, it is simpler and works for any count.)

## 6. Create the IoT policy

One policy serves all boards, scoped by client id (= certificate CN = serial number). Save as `i757-policy.json`, replacing `REGION` and `ACCOUNT_ID` (account id: `aws sts get-caller-identity`):

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": "iot:Connect",
      "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:client/${iot:ClientId}",
      "Condition": { "Bool": { "iot:Connection.Thing.IsAttached": "true" } }
    },
    {
      "Effect": "Allow",
      "Action": ["iot:Publish", "iot:RetainPublish"],
      "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/dev/I757-M/${iot:ClientId}/up/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Subscribe",
      "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topicfilter/dev/I757-M/${iot:ClientId}/dn/*"
    },
    {
      "Effect": "Allow",
      "Action": "iot:Receive",
      "Resource": "arn:aws:iot:REGION:ACCOUNT_ID:topic/dev/I757-M/${iot:ClientId}/dn/*"
    }
  ]
}
```

Two things in there that are **not optional**:

- **`iot:RetainPublish`**: the board publishes its self-description (`up/desc`) and presence (`up/state`, including the LWT) with the MQTT *retain* flag. AWS silently disconnects a client that attempts a retained publish without this permission — a classic "connects, then drops every few seconds" symptom.
- The `Condition` on Connect ties the client id to an attached Thing (§7), so one leaked certificate cannot impersonate another serial number.

```
aws iot create-policy --policy-name i757-boards --policy-document file://i757-policy.json
aws iot attach-policy --policy-name i757-boards --target <certificateArn>
```

## 7. Create a Thing per board and attach the certificate

```
aws iot create-thing --thing-name i757-0002
aws iot attach-thing-principal --thing-name i757-0002 --principal <certificateArn>
```

The Thing name **must equal the certificate CN / serial number** (the Connect condition in §6 depends on it).

## 8. Find your endpoint

```
aws iot describe-endpoint --endpoint-type iot:Data-ATS
```

You get something like `xxxxxxxxxxxxxx-ats.iot.ap-southeast-2.amazonaws.com`. Only the **`-ats`** endpoint works with the Amazon Root CA 1 anchor used below.

## 9. Modify the SDK and rebuild

### 9.1 Add the Amazon trust anchor

Create the `keys/` folder **next to** (as a sibling of) the SDK folder — the build looks one level above the repo so that key material can never be committed:

```
<your workspace>/
  i757-sdk/          <- this repo
  keys/
    aws/
      AmazonRootCA1.pem
```

Download the root CA from <https://www.amazontrust.com/repository/AmazonRootCA1.pem>. (If the file is absent the firmware still builds, but with an empty AWS anchor — the TLS handshake then fails with nothing obviously wrong. Check this first when debugging.) `keys/ca/` is **not needed** for the AWS path; the "STUB certificates" build warning about it is expected and harmless here.

### 9.2 Edit `platform_757/CM7/App/app_cfg.h`

```c
#define CFG_MQTT_AWS_HOST   "xxxxxxxxxxxxxx-ats.iot.ap-southeast-2.amazonaws.com"  /* your §8 endpoint */
#define CFG_DNS_SERVER      "192.168.1.1"    /* a DNS server reachable on YOUR network — see note */
#define CFG_MQTT_PREFERRED  2                /* 2 = AWS is the home broker (boot + fallback target) */
```

- **`CFG_DNS_SERVER`**: AWS is domain-addressed, so the board must resolve the endpoint. The firmware uses this fixed server (it does not take DNS from DHCP). Your router's LAN address is usually right; a public resolver (e.g. `8.8.8.8`) also works if your firewall allows outbound DNS.
- **`CFG_MQTT_PREFERRED 2`**: makes AWS the broker selected at boot *and* the side the auto-fallback returns to. Without this the board boots onto the "public broker" setting and, after any AWS outage, would fall back there permanently.
- `CFG_SE_CUSTOMER_SLOT` (default 2) only needs touching if slot 2 is already used for something else on your boards.
- `CFG_MQTT_PUB_HOST` can stay a placeholder — with `CFG_MQTT_PREFERRED 2` it is never used.

### 9.3 Rebuild and install

Build as usual (Software Manual §2) and install via USB (`usb_fw_push.ps1 -Self`), SWD, or a cloud OTA if the board is still reachable on a previous broker. Once running, the board automatically presents your identity on AWS connections (`identity2` present → customer slot signs the handshake); the factory identity remains in place for anti-clone and vendor connections.

## 10. First connection and verification

1. Power the board on your LAN.
2. AWS Console → **MQTT test client** (left menu) → Subscribe to `dev/#`.
3. Within ~30 s you should see, on `dev/I757-M/<sn>/up/...`:
   - `state`: `{"online":true}` (retained birth message)
   - `desc`: the self-description JSON (retained)
   - `status`: the heartbeat every ~5 s, with `"bkr":"aws"` confirming which broker it is on.
4. Downlink check: publish `beep` to `dev/I757-M/<sn>/dn/cmd` (see `OTA_and_MQTT_User_Guide.md` §5 for the command set) — the board should chirp and report `"beep":1` in the next heartbeat.

On the board side, the USB console (`stat`) shows the broker and connection state live; `id2` confirms which identity is stored.

## 11. PC-side tools (OTA push etc.) against AWS

The PC tools (`ota_push.ps1`, `mota_push.ps1`) are MQTT clients too — AWS must know *them* as well:

1. Generate an operator key pair + certificate signed by your CA (any CN, e.g. `ops-1`) — a plain openssl keypair this time, no chip involved:
   ```
   openssl ecparam -name prime256v1 -genkey -noout -out ops-1.key
   openssl req -new -key ops-1.key -subj "/CN=ops-1" -out ops-1.csr
   openssl x509 -req -in ops-1.csr -CA my-ca.crt -CAkey my-ca.key -set_serial 2001 -days 3650 -out ops-1.crt
   ```
2. Register it as in §5 and attach a policy allowing `iot:Connect` on `client/ops-1*` plus publish/subscribe/receive on the whole `dev/I757-M/*` tree (the operator needs the whole tree; omit the `IsAttached` condition in its policy — no Thing needed).
3. Point the scripts at your endpoint and operator credentials (`-Aws` path; see the parameter block at the top of each script).

Firmware signing is unchanged: OTA still requires the signing private key (`keys/fwsign/`) on the operator machine — broker choice does not relax the anti-counterfeit chain.

## 12. Troubleshooting

| Symptom | Likely cause | Check |
|---|---|---|
| Board never resolves / no TCP to AWS | `CFG_DNS_SERVER` not reachable on your network | Can a PC on the same LAN resolve the endpoint using that server? |
| TLS handshake completes, then immediate disconnect, repeatedly | Certificate not ACTIVE, or policy missing/mismatched | Certificate status in console; enable CloudWatch logging in IoT settings |
| Connects, then drops every few seconds | Retained publish rejected — policy lacks `iot:RetainPublish` | §6 policy; CloudWatch shows RETAIN-class errors |
| Connects, no messages arrive in test client | Publish denied by policy (topic mismatch) | Topic ARNs in §6 vs. actual topics `dev/I757-M/<sn>/up/*` |
| Handshake signature rejected by AWS | Registered cert is not the one the board presents — e.g. `id2-gen` was re-run after signing (new key, stale cert) | `id2` / `id2-cert-show` vs. what you registered; re-sign and `id2-cert` |
| `id2-cert: FAIL pubkey-mismatch` | Cert signed over the wrong public key (another board's, or a pre-regeneration key) | Re-run §4 step 2 with THIS board's current `id2-pub` output |
| `bkr` in heartbeat says `pub`, not `aws` | `CFG_MQTT_PREFERRED` not set to 2, or old firmware still running | `ver` field vs. your build date |
| Handshake fails, nothing in AWS logs at all | Empty AWS anchor — `keys/aws/AmazonRootCA1.pem` absent at build time | Rebuild after §9.1; the configure step warns about stub certificates |

## 13. Scope notes

- Azure IoT Hub / other clouds follow the same "your-CA-signed chip identity + swap the trust anchor + endpoint" pattern but differ in authentication details; this guide covers AWS IoT Core only.
- The vendor-hosted dashboard does not see boards that live in your AWS account. To feed your own dashboard, subscribe with any MQTT client using an operator certificate (§11), or use AWS IoT rules to route messages onward.
- The factory identity (slot 0) is not a secret channel: it only ever connects where `CFG_MQTT_PUB_HOST` points, which is under your control in `app_cfg.h`.
