# FIRMWARE

This directory contains all necessary program files for  Siemens PLC controllers.

## Requirements 

for upload firmware to controller you need:
 - PC with windows
 - SIMATIC STEP 7 (TIA portal V16) program. For uploading project will be enough trial version. You can find installation files at [the bottom of page][db1] with instruction. 
 - Patch cable to connect you PC with controller.

## Instruction
### Install TIA Portal

You need to install TIA Portal. For this download program from official [website][db1]. For this you have to register on site and download all files to one directory. 

![image1][im1]

**Note!** as said in tutorial: *The download is split into multiple files. Please first download all the parts to the same folder and then execute the file with the extension .exe. Afterwards, the parts are merged and you can execute the setup.*

### Work with TIA Portal

First, download files from GitHub [repository][db2]. You should unzip it somewhere.

Open TIA Portal. Open project, which you have just uploaded and unzipped. After that press **project view** button.

![pre_view][im2]

The project will be open. The project tree is located on the left side of window. In the center Device view is located.At this step you can navigate and learn more about project structure.

![structure][im3]

Now please connect main controller PLC S7-1200 to your PC with patch cable. after that press **go online** button. This will start *auto wizard* connection program.

![go_online][im4]

In a new window set up settings as shown on the picture:

![wizard][im5]

***
- Type of the PG/PC interface: PN/IE
- PG/PC interface: Realtek PCIe GbE Family Controller
- Connection to interface/subnet: PN/IE_1
***

After that press ***Start search*** button.

You will see *warning windows* - press **Yes**.

![warning][im6]

Main controller have been connected. Now, if you want, you can change default IP address to different. To do this go at the bottom of window to **Propertiee -> General -> PROFINET interface**:

![IP][im7]

**Be careful!** the same IP address must be in `app/modbus_plc_siemens/scripts/modbus_client_app.py` file at line 18.

![line 18][im8]

That's all with initial setup. 

### Download firmware
Let's download our firmware to the controller. First "disconnect" with controller. Press ***Go offline*** button. It's next to ***Go online*** button.

Now we will compile and download the program. To do this, click ***compile*** button. You could watch logs at the bottom of window.

![compile][im9]

The last step is downloading. Press ***load*** button which located next to ***compile*** button. In the window that opens, click ***load*** button again and observe logs. That's all.

![download][im10]

[db1]: <https://support.industry.siemens.com/cs/document/109772803/simatic-step-7-incl-safety-and-wincc-v16-trial-download?dti=0&lc=en-WW>
[im1]: <../misc/IndustrySupport.jpg>
[db2]: <https://github.com/Multi-Agent-io/lightsout-factory>
[im2]: <../misc/project_view.jpg>
[im3]: <../misc/structure.jpg>
[im4]: <../misc/go_online.jpg>
[im5]: <../misc/wizard.jpg>
[im6]: <../misc/warning.jpg>
[im7]: <../misc/IP.jpg>
[im8]: <../misc/line18.jpg>
[im9]: <../misc/compile.jpg>
[im10]: <../misc/download.jpg>