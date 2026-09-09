# Schéma Protobuf — vla.cpp

`vla.proto` est copié tel quel depuis :
https://github.com/VinRobotics/vla.cpp (chemin source : `src/serving/vla.proto`)

Licence du projet source : Apache 2.0.

`vla_pb2.py` est généré automatiquement par `protoc` à partir de
`vla.proto` — ne pas éditer à la main. Pour régénérer après une mise
à jour du schéma source :

```bash
protoc --python_out=. vla.proto
```

Voir `axis_os/store/sdk/ai/connector.py` (classe `VLACppConnector`)
pour le client Python qui utilise ce schéma, et le protocole ZeroMQ
REQ/REP exact attendu par un serveur `vla-server` compatible.
