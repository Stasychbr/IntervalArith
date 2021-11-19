

clear zz; clear xx; clear yy;
for ii=1:31
    xx(ii)=ii-16;
    for jj=1:31
        yy(jj)=jj-16;
zz(ii, jj) = (ii-16)^2 + (jj-16)^2 - cos(18*(ii-16)) - cos(18*(jj-16));
    end
end
figure
% contour(xx, yy, zz)
meshz(xx, yy, zz)
colormap([.5 .5 .5])
colorbar off
surf(xx, yy, zz)
mesh(xx, yy, zz)
xlabel('\it x_1')
ylabel('\it x_2')
zlabel('\it z')
figure_name_out=strcat('Rastrigin10 BIG', '.pdf')
pwd
print('-dpdf', '-r300', figure_name_out), pwd 
view_Rastrigin10=view

clear zz0; clear xx; clear yy;
NN=200
for ii=1:NN
    xx(ii)=(ii-NN/2)/100;
    for jj=1:NN
        yy(jj)=(jj-NN/2)/100;
zz0(ii, jj) = xx(ii)^2 + yy(jj)^2 - cos(18*xx(ii)) - cos(18*yy(jj));
    end
end
% close all

clear zz0; clear xx; clear yy;
NN=100
box_size=4
for ii=1:NN
    xx(ii)=(ii-NN/2)/(NN/box_size);
    for jj=1:NN
        yy(jj)=(jj-NN/2)/(NN/box_size);
% zz0(ii, jj) = xx(ii)^2 + yy(jj)^2 - cos(18*xx(ii)) - cos(18*yy(jj));
      zz0(ii, jj) = 100*(xx(ii)^2 - yy(jj))^2 + (xx(ii) - 1)^2;
    end
end

%       Y = 100*sqr(sqr(X(1)) - X(2)) + sqr(X(1) - 1);

size(zz0)
[min_zz0 min_zz0_ind]=min(zz0(:))
    % min_flux row, col
    [zz0_min_row, zzo_min_col] = ind2sub(size(zz0), min_zz0_ind)
[max_zz0 max_zz0_ind]=max(zz0(:))
    [zz0_max_row, zzo_max_col] = ind2sub(size(zz0), max_zz0_ind)
xx(zz0_min_row), yy(zzo_min_col)


close all
figure
mesh(xx, yy, zz0)
hold on
for mksz=5:-1:1
plot3(xx(zz0_min_row), yy(zzo_min_col), zz0(zz0_min_row, zzo_min_col),'sk', 'MarkerSize', mksz)
end
% colormap([.5 .5 .5])

% [C, h]=contour(xx, yy, zz0)
% beta=0.1
% brighten(beta);
xlabel('\it x_1')
ylabel('\it x_2')
zlabel('\it z')
colormap([.5 .5 .5])
figure_name_out=strcat('Rastrigin10 SMALL grey', '.pdf')
figure_name_out=strcat('Rosenbrock4 BIG grey', '.pdf')

pwd
print('-dpdf', '-r300', figure_name_out), pwd 

[C, h]=contour(xx, yy, zz0)
[C, h]=contourf(xx, yy, zz0)
colorbar off
caxis([0 3000])
lless=1
xlim([0 3/lless])
ylim([-3/lless 1/lless])
 clabel(C,h)
  clabel(C,h,'LabelSpacing',10,'Color','k')
v = [0,10,20];
clabel(C,h,v)
hold on
plot(xx(zz0_min_row), yy(zzo_min_col), '.k')
 
mesh(xx, yy, zz0)
colormap([.5 .5 .5])
colormap jet
colorbar off
surf(xx, yy, zz0)
mesh(xx, yy, zz0)
xlim([0 2])
ylim([0 2])
caxis([0 30])
zlim([0 100/2])
figure_name_out=strcat('Rosenbrock4 Small grey', '.pdf')

pwd
print('-dpdf', '-r300', figure_name_out), pwd 


figure_name_out=strcat('Rastrigin10 BIG', '.pdf')
pwd
print('-dpdf', '-r300', figure_name_out), pwd 
% view(view_Rastrigin10)

% X2=zeros(10,10);
% for ii=1:size(X2,2)
% 
%     for jj=1:size(X2,2)
%    
%         X2(ii,jj)=ii+10*(jj-1);
% end   
% end
% size(X2)
% 
% Y=zeros(10,10);
% for ii=1:size(X2,1)
% 
%     for jj=1:size(X2,2)
%         X=[ii jj];
%        Y(ii,jj) = sqr(X(1)) + sqr(X(2)) - cos(18*X(1)) - cos(18*X(2));
% %           Y (ii,jj) = 100*sqr(sqr(X(1)) - X(2)) + sqr(X(1) - 1);
% 
% 
% end   
% end
% 
% X=[0:1:10 ];

supX2=[-30; -30]
infX2=[30; 30]
% supX2=[-2.5; -2.5]
% infX2=[2.5; 2.5]
X2=infsup(supX2, infX2)
BOX_str1 = [ ' [' , num2str(supX2(1)), '-', num2str(infX2(1)), ' ]']
BOX_str2 = [ ' [' , num2str(supX2(2)), '-', num2str(infX2(2)), ' ]']
BOX_str = strcat('INI BOX= ', BOX_str1, BOX_str2)

FuncName='Function = Rastrigin10'
FuncName='Function = Rosenbrock4'
% globopt0 
% WorkList(kk).Box - Box at kk iteration
[Z, WorkList ] = globopt0(X2)

Z_str=strcat('GlobalOpt=', num2str(Z))


figure

X1=X2(1); X2=X2(2);
Xplot=[inf(X1) inf(X1) sup(X1) sup(X1) inf(X1) ];
Yplot=[inf(X2) sup(X2) sup(X2) inf(X2) inf(X2)];
line(Xplot, Yplot,'Color','k');
grid on
hold on
for kk=1:length(WorkList)
    Xcur=WorkList(kk).Box
    X1=Xcur(1); X2=Xcur(2);
    Xplot=[inf(X1) inf(X1) sup(X1) sup(X1) inf(X1) ];
Yplot=[inf(X2) sup(X2) sup(X2) inf(X2) inf(X2)];
line(Xplot, Yplot,'Color','k');
end
axis('equal')
hold on
contour(xx,yy,zz0)
colormap([.5 .5 .5])


% % PLOT
% 
% title_name=sprintf('INTERVAL OPTIMIZATION ')
% % Parameters
% title_name1='METOD= SIMPLE GLOBAL OPT' % TOL FUNCTIONAL';
% title_param1=FuncName;
% title_param2=BOX_str;
% title_param3=Z_str %'' %'Lipshitz = Jacobian' %strcat(' t1 =', num2str(t1)) %strcat(' CAMERA DATA =',filename_no_under)
% % Tille name
% title_str=sprintf('INTERVAL OPTIMIZATION \n%s\n%s\n%s\n%s\n%s', title_name1, title_param1, title_param2,title_param3)
% % title_str=strcat(title_name, title_param1,title_param2,title_param3,title_param4)
% title(title_str)
% % PRINT
% figure_name_out=strcat(title_name, '_ ', title_name1, FuncName,'.png')
% figure_name_out=strrep(figure_name_out,':','-')
% figure_name_out=strrep(figure_name_out,'*','-')
% figure_name_out=strrep(figure_name_out, '\','-')
% 
% print('-dpng', '-r300', figure_name_out), pwd 
% %/PLOT
% 
% 
% 
% % 
% close all
% figure
% meshc(Y)
% grid on