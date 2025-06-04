## How to start:

### Set up environment:

1. Setup environment:
   ```bash
   python3 -m venv .venv
   ```

2. Active environment:
   - Unix/macOS
    ```bash
    source .venv/bin/activate
    ```

   - Windows
   ```bash
   .venv\Scripts\activate
   ```

3. Install dependencies
   ```bash
   pip install -r requirements.txt
   ```

### Launch car emulator GUI

4. Move to folder adaptor and launch main.py
   ```bash
   python3 main.py
   ```

Now your GUI is ready to accept connection from Android app. Launch Android app from a device in the same network to connect. Wifi is known to work.