# Exemple d'utilisation d'eCAL pour lire les messages Lidar

1. générer les classes protobuf (voir generate_proto.sh, à adapter pour Windows.)
    - Il sera peut être nécessaire d'installer le "protobuf-compiler".
2. lancer ecal_test.py
3. lancer un replay avec eCAL Player.

# Pour windows
1. ouvrir powershell en admin, puis `set-executionpolicy remotesigned`
2. Télécharger protoc : https://github.com/protocolbuffers/protobuf/releases
3. Remplacer C:\eCAL\bin\protoc.exe par la nouvelle version
4. generate_proto.bat : `protoc -I="." --python_out="./" *.proto --pyi_out="./"`