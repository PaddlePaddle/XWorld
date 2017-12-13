# Atari
## <img src="../../doc/atari.png">

## Game description
Atari games [wiki](https://en.wikipedia.org/wiki/List_of_Atari_2600_games)

## Create
* Python name: ```atari```
* C++ constructor name: ```ArcadeGame```
* Name for the unified C++ simulator interface: ```atari```

## Flags
|**Name**|**Description**|
|:-------|:---------------|
|```pause_screen```|Pause the screen when show_screen() is called, until any key is pressed. (Default: false)|
|```ale_rom```|The Atari ROM file. You need to download the ROMs (.bin files) yourself. For exampe, from [here](https://github.com/npow/atari/tree/master/roms). (Default: "")|
|```context```|How many consecutive frames are used to represent the current sensor input. (Default: 1)|