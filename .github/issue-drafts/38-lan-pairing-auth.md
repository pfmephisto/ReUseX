title: Open rux gui from a phone without disabling the origin policy by hand
labels: gui, enhancement, backend, security

## Problem
On-site is a phone screen, but `rux gui` binds loopback and refuses any
non-loopback `Origin` (SecurityMiddleware). Phase 6 (R10) documents the
workaround — `--bind <LAN-IP> --allow-origin http://<LAN-IP>:<port>` — and
warns that the server has no authentication. Anyone on that network can then
read the project and start pipeline stages. Relaxing the origin check to
"same as Host" is not an option: under DNS rebinding the attacker chooses
the Host.

## Proposed fix
- [ ] A pairing flow: `rux gui --lan` binds the LAN address, prints a
      one-time code (and a QR code in the terminal), and the phone exchanges
      it for a session token; every API request and the `/events` upgrade
      then require the token.
- [ ] Keep loopback requests token-free, so the desktop flow is unchanged.
- [ ] Derive the allowed origin from the bound address, so `--allow-origin`
      is no longer needed for the phone.
- [ ] Replace Sager's recipe panel with "Åbn på telefon" showing the code.

category=CLI estimate=3d
