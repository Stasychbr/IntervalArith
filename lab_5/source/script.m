

## Установить пути к функциям построения интервальной регрессии
addpath(genpath('./m'))

## Загрузить исходные данные
load data/steam.mat
steam

## Определить задачу построения интервальной регрессии 
##     y = X * beta = beta1 + beta2 * x 
## с ограничением beta2 >= 0

x = [2; 4; 10; 12];        # количество затраченного топлива
y = [2; 8.5; 18; 25];        # объем произведенного пара
epsilon = [2; 1.5; 2.5; 1.5];  # верхняя граница ошибки для y_i

X = [ x.^0 x ];                               # матрица значений переменных при beta1 и beta2
lb = [-inf 0];                                # нижние границы beta1 и beta2
irp_steam = ir_problem(X, y, epsilon, lb);



## График интервальных измерений
##figure
##ir_scatter(irp_steam);   
##xlim([0 22])
##ylim([-10 260])
##set(gca, 'fontsize', 12)
##title('Steam generator performance');
##xlabel('Fuel consumption');
##ylabel('Steam quantity');



## Графическое представление информационного множества
figure
ir_plotbeta(irp_steam)
grid on
set(gca, 'fontsize', 12)
xlabel('\beta_1')
ylabel('\beta_2')

## Внешние интервальние оценки параметров модели y = beta1 + beta2 * x 
b_int = ir_outer(irp_steam)



## Вершины информационного множества задачи построения интервальной регрессии
vertices = ir_beta2poly(irp_steam)

## Диаметр и наиболее удаленные вершины информационного множества 
[rhoB, b1, b2] = ir_betadiam(irp_steam)



## Внешние интервальние оценки параметров модели y = beta1 + beta2 * x 
b_int = ir_outer(irp_steam)

## Точечные оценки параметров 
b_maxdiag = (b1 + b2) / 2    # как середина наибольшей диагонали информационного множества

b_gravity = mean(vertices)   # как центр тяжести информационного множества 

b_lsm = (X \ y)'             # методом наименьших квадратов

## Точечные оценки
plot(b_maxdiag(1), b_maxdiag(2), ';Центр макс. диагонали;mo')
plot(b_gravity(1), b_gravity(2), ';Центр тяжести;ks')
plot(b_lsm(1), b_lsm(2), ';МНК;rx')
legend()

figure 
xlimits = [-5 15];
ir_plotmodelset(irp_steam, xlimits)     # коридор совместных зависимостей
hold on
ir_scatter(irp_steam,'b.')              # интервальные измерения
#ir_plotline(b_maxdiag, xlimits, 'r-')   # зависимость с параметрами, оцененными как центр наибольшей диагонали ИМ
#ir_plotline(b_gravity, xlim, 'b--')     # зависимость с параметрами, оцененными как центр тяжести ИМ  
#ir_plotline(b_lsm, xlim, 'b--')         # зависимость с параметрами, оцененными МНК
#ir_scatter(ir_problem(Xp,ypmid,yprad),'ro')

grid on
set(gca, 'fontsize', 12)
xlabel('x')
ylabel('y')

figure 
x_p = [2; 10; -1; 5; 13];
ir_plotmodelset(irp_steam, xlimits)
hold on
ir_scatter(irp_steam,'b.')
hold on 
X_p = [x_p.^0 x_p];
y_p = ir_predict(irp_steam, X_p);
ypmid = mean(y_p,2);
yprad = 0.5 * (y_p(:,2) - y_p(:,1));
ir_scatter(ir_problem(X_p,ypmid,yprad),'k.');
grid on
