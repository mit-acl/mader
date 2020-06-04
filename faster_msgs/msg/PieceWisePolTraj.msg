Header header

float32[] times        # times has n+1 elements [t0,t1,t2,...,tn+1]
CoeffPoly3[] coeff_x        # Array of coeffients. Should have n elements
CoeffPoly3[] coeff_y        # Array of coeffients. Should have n elements
CoeffPoly3[] coeff_z        # Array of coeffients. Should have n elements

float32[] bbox             #bounding box. It has three elements: hx, hy, hz (bbox of size hx x hy x hz)

int32 id                   #id of the dynamic obstacle
bool is_agent              #1 if is an agent, and 0 if it is a static or dynamic obstacle