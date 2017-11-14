# Instructions for creating games
Below we explain the flags that the user can set for each game. If you use Python APIs, these flags are passed in as a dictionary when creating the game (see ```<xworld_path>/python/examples``` for details). If you use C++ APIs, these flags are either set globally from command line with GFlags, or are passed as arguments of the class constructor (see ```<xworld_path>/examples``` for details).

|**ID**|**Python Keyword**|**C++ Class Name**|**Flags**|
|---|---|-----|-----------|
|**a**|```simple_game```|```SimpleGame```|```pause_screen```,```array_size```|
|**b**|```simple_race```|```SimpleRaceGame```|```pause_screen```,```window_width```,```window_height```,<br>```track_type```,```track_width```,```track_length```,<br>```track_radius```,```race_full_manouver```,```random```,<br>```difficulty```|
|**c**|```xworld```|```XWorldSimulator```|```pause_screen```,```conf_path```,```curriculum```,<br>```task_mode```,```task_groups_exclusive```,```context```,```ego_centric```|
|**d**|```atari```|```ArcadeGame```|```pause_screen```,```ale_rom```,```context```|

The meanings of the above flags are listed below. Each flag applies to certain games indicated by the IDs.
* ```pause_screen``` (**a-d**)

    Pause the shown screen when show_screen() is called, until any key is pressed. (Default: false)

* ```array_size``` (**a**)

    The array size of simple game. (Default: 0)

* ```window_width``` (**b**)

    Width of display window for Simple Race. (Default: 0)

* ```window_height``` (**b**)

    Height of display window for Simple Race. (Default: 0)

* ```track_type``` (**b**)

    Track type used in Simple Race: circular track ("circle") or straight track ("straight"). (Default: "straight")

* ```track_width``` (**b**)

    The width of the track in Simple Race. (Default: 0)

* ```track_length``` (**b**)

    The length of the track in Simple Race, if ```track_type``` is "straight". (Default: 0)

* ```track_radius``` (**b**)

    The radius of the circluar track in Simple Race, if ```track_type``` is "circle". (Default: 0)

* ```race_full_manouver``` (**b**)

    In Simple Race, whether to allow turning and moving forward/backward at the same time (true) or not (false). (Default: false)

* ```random``` (**b**)

    In Simple Race, whether to enable random start positiona and facing direction (true) or not (false). (Default: false)

* ```difficulty``` (**b**)

    Difficulty level: "easy" level provides negative rewards when moving away from the center line of the track, and "hard" level only provides rewards when out-of-boundary or reaching finish line. (Default: "easy")

* ```conf_path``` (**c**)

    The conf JSON file for XWorld2D, or the conf XML file for MALMO. (Default: "")

* ```curriculum``` (**c**), can be extended to any of (**a-d**) that employs curriculum learning

    This number indicates the maximum number of games that has curriculum learning. Given any difficulty range that can be quantized by two integers [*low*, *high*], if ```curriculum``` is 0, then the difficulty number is randomly selected from this range; otherwise, the difficulty is linearly increased from *low* to *high* until ```curriculum``` games have been played. (Default: 0)

* ```task_mode``` (**c**)

    This flag has three possible values.

    "arxiv_lang_acquisition": replicate the environment used in arXiv:1703.09831 (used with conf file ```<xworld_path>/games/xworld/confs/navigation.json``` and dictionary ```<xworld_path>/games/xworld/dicts/nav_dict.txt```);

    "arxiv_interactive": replicate the environment used in arXiv:1705.09906 (used with conf file ```<xworld_path>/games/xworld/confs/lang.json``` and dictionary ```<xworld_path>/games/xworld/dicts/lang_dict.txt```);

    "one_channel": integrate the above two environments into a single one.

    (Default: "one_channel")

* ```task_groups_exclusive``` (**c**), can be extended to any of (**a-d**) that incorporates a teacher

    In XWorld2D, whether the agent handles multiple tasks simultaneously (false) or not (true). (Default: true)

* ```context``` (**c-d**)

    How many consecutive frames are used to represent the current state. (Default: 1)

* ```ego_centric``` (**c**)

    Whether the training image is ego-centric or not. When true, the actual world dimensions of the training image will be (2*h-1) x (2*w-1). (Default: true)

* ```ale_rom``` (**d**)

    The Atari ROM file path. You need to download the ROMs (.bin files) yourself. (Default: "")