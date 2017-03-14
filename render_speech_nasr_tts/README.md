### Installation

1. Install requirement packages
    $ sudo apt install libsamplerate0-dev libportaudio-ocaml-dev


2. Download and install from Github
    $ git clone https://github.com/NuanceDev/ndev-python-http-cli.git
    $ sudo python setup.py install

3. Testing
    $ python
    >>> import ndev


### Register Nuance Developer Account

1. Visit https://developer.nuance.com/public/index.php?task=home
2. Register and check the Sandbox Credentials (https://developer.nuance.com/public/index.php?task=credentials)
3. You can get informations as below:
    * HTTP Interface Applications
    - ASR URI
    - TTS URI
    - App Id
    - App Key


### Create config file
1. Create the directory for saving credentials information.
2. Create credentials.json file and fill the information as below.
    ```
    {
        "appId": "AppId",
        "appKey": "AppKey",
        "asrUrl": "https://dictation.nuancemobility.net",
        "asrEndpoint": "/NMDPAsrCmdServlet/dictation",
        "ttsUrl": "https://tts.nuancemobility.net:443",
        "ttsEndpoint": "/NMDPTTSCmdServlet/tts"
    }
    ```
