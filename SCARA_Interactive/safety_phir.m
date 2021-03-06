function [thres,vet]=safety_phir(D,BJ,margin, kphir, xi, dxi)
n=size(BJ,2);
P1=eye(2*n);
for i=n+1:2*n
    P1(i,i)=0;
end
P2=[zeros(n) 0.5*eye(n);0.5*eye(n) zeros(n)];
dnext=sqrt(D'*P1*D);
thres=margin*dnext-dnext^3-D'*P2*D + kphir*(dxi' *xi*dxi);
vet=2*D'*P2*BJ;
end