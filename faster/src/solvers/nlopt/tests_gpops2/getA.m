function A=getA(B,R)
    global deg_pol t



    W=[];
    %Insert half of the polynomials
    for i=1:((deg_pol+1)/2)
        pol=-B(i)*(t-1);
        for j=1:(deg_pol-1)/2
            pol=pol*((t-R(i,j))^2);
        end
        W=[W;pol];
    end

    %Insert the other half
    W=[W;subs(W,t,-t)];
    
    
    %Create the A matrix:
    A=[];
    for i=1:length(W)
        tmp=coeffs(W(i),t,'All');
        
        tmp=[zeros(1,deg_pol+1-length(tmp)) tmp];
        
        A=[A; tmp];
    end
   
%     vpa(W)
%      vpa(A)
   
    
end