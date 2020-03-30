from ncpol2sdpa import *

X = generate_variables('x', 2)
obj = X[0]
inequalities = [X[1]]
sdp = SdpRelaxation(X)
sdp.get_relaxation(2, objective=obj, inequalities=inequalities,
                   chordal_extension=True)
