function draw_sphere_f(centerX, centerY, radius)
    
    [Xs, Ys, Zs] = sphere(100); 
    Xs = Xs * radius + centerX; 
    Ys = Ys * radius + centerY; 
    Zs = Zs * radius; 
    surf(Xs, Ys, Zs, 'EdgeColor', 'none', 'FaceColor', 'b');
end
