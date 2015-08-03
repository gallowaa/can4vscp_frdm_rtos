<h1> MQX RTOS port of VSCP firmware to Kinetis </h1>
<p> This is one of the pieces of the "EasyIoTwithVSCP" project on the Freescale Community. The first supported platform is the FRDM-K64F, however since the Kinetis SDK is used it should be highly portable to other Kinetis devices with a FlexCAN peripheral </p>

<h2> Project Requirements</h2>

Note that this repository is a work in progress.

<h3> 1. You will want the VSCP firmware & software: </h3>
<ul>
	<li> only the firmware is required to compile this project </li>
    <li><code>git clone https://github.com/grodansparadis/vscp_software</code></li>
    <li><code>git clone https://github.com/grodansparadis/vscp_firmware</code></li>
</ul>

<h3> 2. Install the following free tools from Freescale: </h3>
<ul>
    <li> Freescale's eclipse based IDE <a href="http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KDS_IDE">Kinetis Design Studio 3.0.0</a></li>
    <li> Software Development Kit for Kinetis version 1.2 <a href="http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=KINETIS-SDK">Kinetis SDK 1.2</a></li>
</ul>

<h5> Setup the Freescale tools: </h5>
<ul>
<li> Accept the default KSDK installation path, ex. "C:/Freescale" on a Windows machine</li>
<li> An update is needed to make the IDE KSDK compatible. To install:</li>
<li><code> Help > Install New Software > Add > Archive > Browse C:/Freescale/KSDK_1.2.0/tools/eclipse_update/KSDK_1.2.0_Eclipse_Update.zip </code></li>

<li> When using the KSDK, we need to link to a platform library which contains hardware and operating system abstraction layers (HAL & OSA), peripheral drivers, and startup code. To Build the platform library: </li>
<li><code> File > Import > General > Existing Projects into Workspace > Next > Browse
C:/Freescale/KSDK_1.2.0/lib/ksdk_platform_lib/kds/<device_name></code></li>

<li> These additional MQX libraries must also be built: 
	<ol>
		<li>MQX Library <code> ${KSDK_PATH}/rtos/mqx/mqx/build/kds/mqx_<device></code></li>
		<li>MQX Platform Library <code> ${KSDK_PATH}/lib/ksdk_mqx_lib/kds/<device></code></li>
		<li>MQX Standard Library <code> ${KSDK_PATH}/rtos/mqx/mqx_stdlib/build/kds/mqx_stdlib_<device></code></li>
	</ol>
</li>	
</ul>

<h3> 3. Additional Info </h3>
<ul>
<li> To install free debugger + serial emulator firmware on FRDM board 
<a href="http://mcuoneclipse.com/2014/04/27/segger-j-link-firmware-for-opensdav2/">MCU on Eclipse</a></li>
<li>Document for setting up a project workspace
<a href="https://community.freescale.com/docs/DOC-103405">Creating a new MQX Project in KDS DOC-103405 </a></li>
</ul>

![VSCP logo](http://vscp.org/images/vscp_logo.jpg)


