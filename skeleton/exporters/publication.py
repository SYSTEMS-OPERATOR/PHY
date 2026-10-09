"""Stage complete packages and restore previous files on ordinary I/O failure."""
import os
import logging
from pathlib import Path
import shutil
import tempfile
from typing import Dict


class PublicationError(OSError):
    def __init__(self, cause: OSError, rollback_errors, recovery_files):
        super().__init__(str(cause))
        self.rollback_errors = rollback_errors
        self.recovery_files = recovery_files


def publish_files(payloads: Dict[Path, bytes], manifest: Path) -> None:
    """Publish the manifest last; callers/readers must verify its file hashes.

    This is not a multi-directory atomic rename. Serialization finishes before
    staging; ordinary write/replace failures roll back. A process/power failure
    can interrupt publication, so an unverified manifest is never acceptance.
    Callers must provide exclusive access to the destination package.
    """
    targets = list(payloads)
    targets.remove(manifest)
    targets.append(manifest)
    staged = {}
    backups = {}
    written = []
    recovery = []
    try:
        for target in targets:
            target.parent.mkdir(parents=True, exist_ok=True)
            fd, name = tempfile.mkstemp(prefix=f".{target.name}.stage-", dir=target.parent)
            staged[target] = Path(name)
            with os.fdopen(fd, "wb") as stream:
                stream.write(payloads[target])
                stream.flush()
                os.fsync(stream.fileno())
            if target.exists() or target.is_symlink():
                fd, name = tempfile.mkstemp(prefix=f".{target.name}.backup-", dir=target.parent)
                os.close(fd)
                backup = Path(name)
                backup.unlink()
                backups[target] = backup
                shutil.copy2(target, backup, follow_symlinks=False)
        for target in targets:
            os.replace(staged[target], target)
            written.append(target)
    except OSError as cause:
        rollback_errors = []
        for target in reversed(written):
            try:
                if target in backups:
                    os.replace(backups[target], target)
                else:
                    target.unlink()
            except OSError as error:
                rollback_errors.append({"file": target.name, "issue": str(error)})
                if target in backups:
                    recovery.append({"target": str(target), "backup": str(backups[target])})
        raise PublicationError(cause, rollback_errors, recovery) from cause
    finally:
        retained = {item["backup"] for item in recovery}
        for path in list(staged.values()) + list(backups.values()):
            if str(path) not in retained:
                try:
                    path.unlink(missing_ok=True)
                except OSError:
                    logging.getLogger(__name__).warning("Could not remove export temporary file: %s", path)
