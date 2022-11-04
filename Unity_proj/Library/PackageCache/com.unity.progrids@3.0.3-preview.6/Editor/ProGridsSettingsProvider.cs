using UnityEditor.SettingsManagement;

namespace UnityEditor.ProGrids
{
    sealed class Pref<T> : UserSetting<T>
    {
        public Pref(string key, T value, SettingsScope scope = SettingsScope.Project)
            : base(ProGridsSettings.instance, key, value, scope)
        { }

        public Pref(Settings settings, string key, T value, SettingsScope scope = SettingsScope.Project)
            : base(settings, key, value, scope) { }
    }
    
    static class ProGridsSettings
    {
        internal const string k_PackageName = "com.unity.progrids";

        static Settings s_Instance;

        internal static Settings instance
        {
            get
            {
                if (s_Instance == null)
                {
                    s_Instance = new Settings(k_PackageName);
                }

                return s_Instance;
            }
        }

        public static void Save()
        {
            instance.Save();
        }

        public static void Set<T>(string key, T value, SettingsScope scope = SettingsScope.Project)
        {
            instance.Set<T>(key, value, scope);
        }

        public static T Get<T>(string key, SettingsScope scope = SettingsScope.Project, T fallback = default(T))
        {
            return instance.Get<T>(key, scope, fallback);
        }

        public static bool ContainsKey<T>(string key, SettingsScope scope = SettingsScope.Project)
        {
            return instance.ContainsKey<T>(key, scope);
        }

        public static void Delete<T>(string key, SettingsScope scope = SettingsScope.Project)
        {
            instance.DeleteKey<T>(key, scope);
        }
    }
    
    static class ProGridsSettingsProvider
    {
        const string k_PreferencesPath = "Preferences/ProGrids";

#if UNITY_2018_3_OR_NEWER
        [SettingsProvider]
        static SettingsProvider CreateSettingsProvider()
        {
            var provider = new UserSettingsProvider(k_PreferencesPath,
                ProGridsSettings.instance,
                new[] { typeof(ProGridsSettingsProvider).Assembly });

            return provider;
        }

#else

        [NonSerialized]
        static UserSettingsProvider s_SettingsProvider;

        [PreferenceItem("ProGrids")]
        static void ProGridsPreferencesGUI()
        {
            if (s_SettingsProvider == null)
            {
                s_SettingsProvider = new UserSettingsProvider(ProGridsSettings.instance, new[] { typeof(ProGridsSettingsProvider).Assembly });
            }

            s_SettingsProvider.OnGUI(null);
        }
#endif
    }
}