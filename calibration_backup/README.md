# Camera calibration backup

Per-unit factory calibration for both D435i units, captured **2026-08-21** at
firmware 5.17.3.10.

| file | what it is |
|---|---|
| `<serial>_calibration_table.bin` | Raw 512-byte on-device table. **This is the restorable artefact.** |
| `<serial>_calibration.yaml` | Human-readable record: 31 stream profiles with intrinsics, 12 inter-stream extrinsic pairs, device metadata, and the table's fingerprint. |

Verified genuinely per-unit — the two tables differ, so this is real factory
calibration rather than a shared default.

| serial | fnv1a | firmware |
|---|---|---|
| `261722073945` | `867831bc4e4b8603` | 5.17.3.10 |
| `261822074567` | `efc645a09ae28049` | 5.17.3.10 |

## Regenerate

```bash
build/camera/backup_calibration calibration_backup
```

Worth re-running after any firmware update, and before ever running an on-chip
self-calibration.

## Restoring

**Try the device's own factory reset first.** The camera keeps a separate
factory copy that survives user writes, so a bad self-calibration is usually
recoverable without touching these files at all
(`reset_to_factory_calibration()` in librealsense).

These backups are for the case that reset cannot cover: restoring a *specific*
table, including a self-calibration result you wanted to keep and have since
overwritten.

```bash
build/camera/backup_calibration --restore 261722073945 \
    calibration_backup/261722073945_calibration_table.bin --yes-write-eeprom
```

The flag is deliberately awkward. This writes EEPROM and the tool cannot undo
it; it refuses an empty file and prints the table's fingerprint first so you can
check it matches the record above.

## Why bother

The calibration table is written per unit on Intel's rig and is not reproducible
locally. If it were corrupted, **nothing downstream would report an error** —
depth would simply be wrong, and every module would stay perfectly
self-consistent while producing wrong answers. That is the same failure class as
a mis-measured calibration board: invisible precisely because everything agrees.
