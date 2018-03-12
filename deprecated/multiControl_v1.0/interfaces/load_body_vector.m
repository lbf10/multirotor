function [corpo] = load_body_vector
% Esta função cria uma seta azul
% Roberto Santos Inoue
% Data: 18/03/2012
l_x = 0.005;
l_y = 0.005;
l_z = 0.8;

corpo.vertices(1,:) = [0, 0, l_z]; 
corpo.vertices(2,:) = [-l_x, -l_y, 0]; 
corpo.vertices(3,:) = [l_x , -l_y, 0]; 
corpo.vertices(4,:) = [l_x, l_x, 0]; 
corpo.vertices(5,:) = [-l_x, l_y, 0];
corpo.faces = [1 2 3; 1 3 4; 1 4 5; 1 5 2; 2 3 4; 4 5 2];
corpo.n = size(corpo.vertices,1);

corpo.objeto = patch('Vertices',corpo.vertices,...
    'Faces',corpo.faces,'EdgeColor','blue');
end