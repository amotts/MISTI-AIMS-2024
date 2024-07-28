close all

syms t w r


% Constants from the paper
G = 10^7; % N/m^2 Shear Modulus of seabed solid
B = 10^7; % N/m^2 Bulk modulus of porous fluid
k = 10^(-10); % m^3s/kg Soil Permeability Coefficient
n = 0.3; % seabed porosity coefficient
mu = 1.325; % t/(ms) water viscosity coefficient
gamma = 10^4; % specific weight of water
m = 0.9; % ratio of solid skeleton elasticity to fluid elasticity
nu = 0.333; % Poisson's ration of solid fluid systems

speeds = [10^(-3), 10^(-4), 10^(-5), 10^(-6), 10^(-7)];

time = 10000;
max_F = zeros(length(speeds),10);
max_T = zeros(length(speeds), 10);
for radius = 0:9
    output = zeros(time*10,length(speeds));
    
    for w_step = 0:length(speeds)-1
        w = speeds(w_step+1);
        for t_step = 0:time*10
            t= t_step/10;
            % r = 1;
            r = (radius +1)/10;
        
            %kappa = B/gamma;
            m = n*G/((1-2*nu)*B);
        
            alpha = ((1+m)/m)*((G/k)/(pi*m*(1-2*nu)+((pi*(1-2*nu))/(2*(1-nu)))))^(1/2);
        
            f_0 = (6*mu/(alpha*w^3))^0.5*t^(-7/4);
        
            if (f_0*r) > 800
                func = @(tau) (1-(2/(f_0*r)*exp(-1/(2*f_0*r))))*(1./((t-tau).^0.5));
                %disp('using extimation')
            else
                func = @(tau) (1-(2*besseli(1, f_0*r))./(f_0*r*besseli(0, f_0*r)))*(1./((t-tau).^0.5));
                %disp('using exact')
            end
        
            F = pi*r.^2 *alpha*w*integral(func, 0,t);
            output(t_step+1, w_step+1) = F;
        end
    end
    [max_F(:, radius+1),max_T(:, radius+1)] = max(output);
end

plot_time = 0:0.1:time;
plot_time = plot_time';
figure(1)

% loglog(plot_time, output)
% % hold on
% legend('10^(-3)','10^(-4)','10^(-5)','10^(-6)', '10^(-7)')
% xlim([0.1,1000])
% ylim([10, 10^6.5])
% grid on
% xlabel("Time (s)")
% ylabel("Force (N)")
% disp(max(output))

figure(2)
semilogy(1:10, max_F)
legend('10^(-3)','10^(-4)','10^(-5)','10^(-6)', '10^(-7)')
xlim([0, 11])
ylim([10, 10^9])
grid on
xlabel("Radius (m)")
ylabel("Max Force (N)")

figure(3)
semilogy(1:10, max_T)
legend('10^(-3)','10^(-4)','10^(-5)','10^(-6)', '10^(-7)')
xlim([0, 11])
ylim([10, 10^4])
grid on
xlabel("Radius (m)")
ylabel("Time of Max Force (s)")