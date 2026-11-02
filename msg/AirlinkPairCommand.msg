uint64 timestamp
uint64 timestamp_sample

# Published by the pair controller. The airlink module forwards follower
# commands and the controller mux applies master commands locally.
bool valid
float32 collective

float32[2] master_motors
float32[3] master_servos
float32[2] follower_motors
float32[3] follower_servos
