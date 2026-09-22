import importlib.util
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("build", Path(__file__).parents[1] / "tools/build.py")
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)


class ReleaseGuards(unittest.TestCase):
    def test_tag_must_match_version_and_default_branch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(build, "ROOT", root):
                def git(*args):
                    return subprocess.run(["git", *args], cwd=root, check=True,
                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                git("init", "--initial-branch=master")
                git("config", "user.name", "Test")
                git("config", "user.email", "test@example.invalid")
                (root / "ArduinoTec-Pedals").mkdir()
                (root / "ArduinoTec-Pedals/version.h").write_text('#define PEDALS_VERSION "0.2.0"\n')
                git("add", ".")
                git("commit", "-m", "Release base")
                git("update-ref", "refs/remotes/origin/master", "HEAD")
                git("tag", "v0.2.0")
                build.validate_release("v0.2.0")
                with self.assertRaisesRegex(ValueError, "match"):
                    build.validate_release("v0.2.1")
                with self.assertRaisesRegex(ValueError, "version tag"):
                    build.validate_release("--invalid")
                git("checkout", "-b", "unmerged")
                (root / "unmerged.txt").write_text("not on default branch\n")
                git("add", ".")
                git("commit", "-m", "Unmerged change")
                git("tag", "--force", "v0.2.0")
                with self.assertRaises(subprocess.CalledProcessError):
                    build.validate_release("v0.2.0")
