### Required python package

- pygame
- urllib


### Setup

1. Register at https://developers.naver.com/main/
2. Request to register application at https://developers.naver.com/apps/#/register?defaultScope=vocauth
3. You can get client_id and client_secret
4. Create configuration file in arbitrary location (recommend config directory in this package) for launch this node as below.
```
    {
        "client_id": "YMQz1gXDiMznmDjdrDRL",
        "client_secret": "zVl_ZZLr67",
        "language": "kr",
        "speaker": "jinho",
        "speed": 0
    }
```

### Configuration

- language: "kr", "en"
- speaker: "kr" -> "jinho", "minji", "en" -> "clara", "matt"
