# VNC Configuration for Nvidia Jetson Nano

1. Open Vino config file.

    ```bash
    sudo nano /usr/share/glib-2.0/schemas/org.gnome.Vino.gschema.xml
    ```

2. Add following code below the last block.

    ```xml
    <key name='enabled' type='b'>
      <summary>Enable remote access to the desktop</summary>
      <description>
        If true, allows remote access to the desktop via the RFB
        protocol. Users on remote machines may then connect to the
        desktop using a VNC viewer.
      </description>
      <default>true</default>
    </key>
    ```

3. Save and close the file. Now compile the file.

    ```bash
    sudo glib-compile-schemas /usr/share/glib-2.0/schemas
    ```

4. Open Startup Applications Preferences and add VNC to the list.

    - Click Add.
    - Type Name as ***Vino***
    - Type Command as ***/usr/lib/vino/vino-server***
    - Click Save.

5. Open terminal and run the following commands.

    ```bash
    gsettings set org.gnome.Vino require-encryption false
    gsettings set org.gnome.Vino prompt-enabled false
    ```
