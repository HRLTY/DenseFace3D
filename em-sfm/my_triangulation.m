m = size(P3, 1);
t = m / 3;
set(gca,'YDir','Reverse');
figure;
for i = 1 : t
    if i > 1
        delete(hs);
    end
    x = P3(i,:)';
    y = P3(i+t, :)';
    z = P3(i+2*t, :)';
%     x = x(end:-1:1);
%     y = y(end:-1:1);
%     z = z(end:-1:1);
    TRI = delaunayTriangulation(x,y);
    ax = axes;
    hs = triplot(TRI);
    
    set(ax, 'Ydir', 'reverse')
    pause(0.08);
end

