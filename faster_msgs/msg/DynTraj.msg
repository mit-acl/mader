Header header
string[] function #Theis field is only used by the obstacles (i.e. when is_agent==false)
float32[] bbox             #bounding box. It has three elements: hx, hy, hz (bbox of size hx x hy x hz)
geometry_msgs/Vector3 pos  #current position (x,y,z)
int32 id                   #id of the dynamic obstacle
bool is_agent              #1 if is an agent, and 0 if it is a static or dynamic obstacle

#These fields are only used by the agents(i.e. when is_agent==true)
PieceWisePolTraj pwp