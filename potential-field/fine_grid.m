uuu=zeros(N,M);
vvv=zeros(N,M);

for ijk=1:(n_road+n_obs)
    uuu(road(ijk,1),road(ijk,2))=u(ijk);
    vvv(road(ijk,1),road(ijk,2))=v(ijk);
end

quiver(yy,-xx,vvv,-uuu)


uuuq= interp2(xx,yy,uuu,xq,yq);
vvvq= interp2(xx,yy,vvv,xq,yq);

quiver(yq,-xq,vvvq,-uuuq)