# Total latency
+ Quest orientatie naar quest_bridge_node
+ quest_bridge_node naar arduino_parallel
    arduino_parallel echot seq naar quest via camera feed, 
    latency = t_eind - t_begin

+ arduino_parallel orientatie input naar arduino motor command
+ arduino motor command naar camera beweging
+ camera feed sturen naar zed_sender

+ zed_sender camera feed sturen naar quest