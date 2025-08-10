function [x,y] = update_sys(A,B,C,Q,R,x,u)
    x = A * x + B * u + sqrtm(Q) * randn(size(x));
    y = C * x +  sqrtm(R)  * randn(size(C,1),1);
end