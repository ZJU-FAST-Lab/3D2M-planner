using System;
using System.IO;
using Newtonsoft.Json;
using Unity.Services.Core.Internal;
using UnityEngine;
using NotNull = JetBrains.Annotations.NotNullAttribute;

namespace Unity.Services.Core.Telemetry.Internal
{
    class FileCachePersister<TPayload> : ICachePersister<TPayload>
        where TPayload : ITelemetryPayload
    {
        public FileCachePersister(string fileName)
        {
            FilePath = Path.Combine(Application.persistentDataPath, fileName);
        }

        public string FilePath { get; }

        public bool CanPersist { get; } = IsFilePersistenceAvailable();

        static bool IsFilePersistenceAvailable()
        {
            // Switch requires a specific setup to have write access to the disc so it won't be handled here.
            return Application.platform != RuntimePlatform.Switch
                && !string.IsNullOrEmpty(Application.persistentDataPath);
        }

        public void Persist(CachedPayload<TPayload> cache)
        {
            var serializedEvents = JsonConvert.SerializeObject(cache);
            File.WriteAllText(FilePath, serializedEvents);
        }

        public bool TryFetch(out CachedPayload<TPayload> persistedCache)
        {
            if (!File.Exists(FilePath))
            {
                persistedCache = default;
                return false;
            }

            try
            {
                var rawPersistedCache = File.ReadAllText(FilePath);
                persistedCache = JsonConvert.DeserializeObject<CachedPayload<TPayload>>(rawPersistedCache);
                return true;
            }
            catch (Exception e)
            {
                CoreLogger.LogException(e);
                persistedCache = default;
                return false;
            }
        }

        public void Delete()
        {
            if (File.Exists(FilePath))
            {
                File.Delete(FilePath);
            }
        }
    }
}
