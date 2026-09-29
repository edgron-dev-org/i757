# certs/ — public material committed with the source

Nothing in this directory is secret. It is what a fresh checkout needs so that a firmware you build
yourself connects to the Edgron cloud and accepts Edgron firmware releases:

| File | What it is | Used for |
|---|---|---|
| `ca.crt` | Edgron Industry757 Root CA certificate (ECC P-256, public) | The board verifies the broker's server certificate against it (mTLS trust anchor). |
| `fwsign.pub` | Edgron firmware-signing public key (ECDSA P-256) | The board verifies `ota-sign` signatures on Edgron releases. Your own key is registered separately on the board (`fwkey2 set`). |

`app_tls.cmake` reads these when no `keys/` directory is present next to the repository. The private
halves (CA key, signing key, per-board certificates) never leave Edgron; the board's own mTLS identity
comes from its ATECC608A and the identity partition written at the factory, so no device certificate
is needed here either.

If Edgron rotates the CA or the signing key, this directory is updated in the same commit.
