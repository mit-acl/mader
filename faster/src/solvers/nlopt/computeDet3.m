function result=computeDet3(A)

if(size(A,1)~=3 | size(A,2)~=3)
    error('Matrix should be 3x3')
end

result=A(1,1)*A(2,2)*A(3,3)  +  A(1,2)*A(2,3)*A(3,1) +  A(2,1)*A(3,2)*A(1,3) +...
             -A(1,3)*A(2,2)*A(3,1) -  A(1,2)*A(2,1)*A(3,3) -  A(1,1)*A(2,3)*A(3,2);
end