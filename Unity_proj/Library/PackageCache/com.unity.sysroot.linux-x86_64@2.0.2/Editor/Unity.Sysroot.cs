using System;
using System.IO;
using System.Collections.Generic;
using UnityEngine;
using NiceIO.Sysroot;
using UnityEditor.Il2Cpp;

namespace UnityEditor.Il2Cpp
{
    /// <summary>
    /// Sysroot package for Linux x86_64 targets
    /// </summary>
    public class SysrootLinuxX86_64: SysrootPackage
    {
        private string _packageName           => "com.unity.sysroot.linux-x86_64";
        /// <summary>
        /// Name of package
        /// </summary>
        public override string Name           => _packageName;
        /// <summary>
        /// Name of target platform
        /// </summary>
        public override string TargetPlatform => "linux";
        /// <summary>
        /// Name of target architecture
        /// </summary>
        public override string TargetArch     => "x86_64";

        private string _payloadVersion => "9.1.0-2.17-v0_608efc24a3b402ec57809211b16a6d32d519f891d4038e1fc8509fe300c395b2-1";
        private string _payloadDir;
        private string _target => "x86_64-glibc2.17-linux-gnu";

        private NPath _sysrootInstallDir;

        /// <summary>
        /// Initialize sysroot
        /// </summary>
        public SysrootLinuxX86_64()
        {
            _payloadDir = $"linux-x86/{_payloadVersion}";
            RegisterPayload(_packageName, _payloadDir);
            _sysrootInstallDir = PayloadInstallDirectory(_payloadDir);
        }

        /// <summary>
        /// Get sysroot install directory
        /// </summary>
        /// <returns>Directory where sysroot is installed</returns>
        public string SysrootInstallDirectory()
        {
            return _sysrootInstallDir.InQuotes(SlashMode.Native);
        }

        /// <summary>
        /// Arguments supplied to il2cpp.exe
        /// </summary>
        /// <returns>Next argument to il2cpp.exe</returns>
        public override IEnumerable<string> GetIl2CppArguments()
        {
            var sysroot = PayloadInstallDirectory(_payloadDir);

            yield return $"--sysroot-path={sysroot}";
            yield return $"--compiler-flags=\"-target {_target}\"";
            yield return $"--linker-flags=\"-target {_target}\"";
        }

#if !IL2CPP_LEGACY_API_PRESENT
        public override string GetSysrootPath()
        {
            return PayloadInstallDirectory(_payloadDir).ToString();
        }

        public override string GetToolchainPath()
        {
            return null;
        }

        public override string GetIl2CppCompilerFlags()
        {
            return $"-target {_target}";
        }

        public override string GetIl2CppLinkerFlags()
        {
            return $"-target {_target}";
        }
#endif
    }
}
