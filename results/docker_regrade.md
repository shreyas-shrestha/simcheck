| Model | Task | Seeds | Reward (subprocess) | Reward (Docker) | Accepted (Docker) | Isolation |
|---|---|---|---|---|---|---|
| argon-medium | prior_needed | eval | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | prior_needed | fresh | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | standard | eval | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | standard | fresh | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | trap_frame | eval | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | trap_frame | fresh | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | trap_units | eval | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
| argon-medium | trap_units | fresh | 1.0 | 1.0 | True | docker --network none, read-only rootfs, public/ copy mounted ro, no env vars |
