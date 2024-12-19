# AEB - Automatic emergency brake

If the time to collision (ttc) is below a certain threshold, the node publishes True on the /emergency_brake topic. As soon as the distance to the nearest obstical is above a threshold False is send. 