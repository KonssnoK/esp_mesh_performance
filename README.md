# esp_mesh_performance
ESP MESH network performance firmware for dev kit C using fixed ROOT-nodes architecture

this code is based on the code "internal_communication" example from ESP-IDF.

How does this work?

There are 2 folders:
- mesh_client
	- Firmware to put on each single device which is not the root.
	- The firmware will just respond to any unicast packet.
- mesh_server
	- Where the magic happens -> Firmware to put on the ROOT device.
	- ROOT needs hotspot connection to a router in order to start the mesh network
	- Hooking to the mesh callbacks send a packet to each node in the network and wait for the answer. 
	- Upon answer reception computes round trip time (RTT)
	- Every 5 seconds dumps the network tree
	- Show error if a packet request is not answered in ~30sec (depends on network state)

To build:
In each folder execute:
- make menuconfig -> Set COM port + Router SSID + Password + Mesh Password
- make flash
- make monitor

Example output:
```W (68716) mesh_root: Layer 0 means unknown! ME(3305)
W (68716) mesh_root: layer 0 (0<-3304)
W (68716) mesh_root: layer 2 (3305<-387C) (3305<-2ECC) (3305<-3120) (3305<-2ECC) (3305<-3120)
W (68726) mesh_root: layer 3 (387D<-4620)
W (68726) mesh_root: layer 4 (40C1<-38FC) (4621<-40C0)
I (68916) mesh_root: 2ECC: RTT 75. Elapsed 211ms
I (68956) mesh_root: 3120: RTT 76. Elapsed 245ms
I (69076) mesh_root: 4620: RTT 74. Elapsed 393ms
I (69366) mesh_root: 40C0: RTT 73. Elapsed 986ms
I (69496) mesh_root: 38FC: RTT 72. Elapsed 1180ms
I (70816) mesh_root: 387C: RTT 77. Elapsed 9ms
I (71196) mesh_root: 3120: RTT 82. Elapsed 6ms
I (71196) mesh_root: 2ECC: RTT 81. Elapsed 16ms
I (71726) mesh_root: 4620: RTT 80. Elapsed 656ms
I (72296) mesh_root: 40C0: RTT 79. Elapsed 1250ms
I (72396) mesh_root: 38FC: RTT 78. Elapsed 1358ms
I (72396) mesh_root: 38FC: RTT 78. Elapsed 1359ms
I (73496) mesh_root: 3120: RTT 88. Elapsed 44ms
I (73536) mesh_root: 387C: RTT 83. Elapsed 278ms
I (73536) mesh_root: 2ECC: RTT 87. Elapsed 124ms
I (73696) mesh_root: 4620: RTT 86. Elapsed 419ms
I (74036) mesh_root: 40C0: RTT 85. Elapsed 763ms
I (74046) mesh_root: 38FC: RTT 84. Elapsed 777ms
I (75586) mesh_root: 2ECC: RTT 93. Elapsed 7ms
```