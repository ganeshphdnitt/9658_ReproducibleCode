%% ========================================================================
% Hybrid GEO-PO Algorithm for PI Controller Optimization
% Author: Ganesh babu M
% Date: Jan 2025
% Description:
%   This script implements the Hybrid Golden Eagle Optimizer - Puma Optimizer
%   algorithm to tune a PI controller's Kp and Ki gains by minimizing a 
%   given objective function. The results include convergence plots and
%   performance metrics.
%% ========================================================================

clc; clear; close all;

%% Problem Definition
global Kp Ki;

% Design Variables: [Kp, Ki]
numVars = 2;

% Variable Bounds
lowerBounds = [0.001, 0.001]; % Minimum for Kp, Ki
upperBounds = [1, 100];       % Maximum for Kp, Ki

% Population and Generations
popSize = 30;
maxGen = 30;

% Objective Function (User-defined, example: Sphere Function)
objFcn = @PI_Controller_Objective;

% Algorithm Parameters
alpha = 2.0;   % Exploration factor (GEO)
beta = 1.5;    % Exploitation factor (GEO)
gamma = 0.5;   % Hybrid weighting factor (PO influence)

% Initialize Population
pop = zeros(popSize, numVars);
fitness = zeros(popSize, 1);

for v = 1:numVars
    pop(:, v) = lowerBounds(v) + rand(popSize, 1) .* (upperBounds(v) - lowerBounds(v));
end

% Evaluate Initial Fitness
for i = 1:popSize
    fitness(i) = objFcn(pop(i, :));
end

% Convergence History
bestFitHistory = zeros(maxGen, 1);
bestSolHistory = zeros(maxGen, numVars);

% Main Loop
fprintf('=== Hybrid GEO-PO Algorithm ===\n');
ticAlgo = tic;

for gen = 1:maxGen
    [bestFit, bestIdx] = min(fitness);
    [worstFit, worstIdx] = max(fitness);

    Best = pop(bestIdx, :);
    Worst = pop(worstIdx, :);

    % New Population
    newPop = zeros(popSize, numVars);
    newFitness = zeros(popSize, 1);

    for i = 1:popSize
        for j = 1:numVars
            r1 = rand; r2 = rand;

            % GEO exploration/exploitation
            if rand < 0.5
                newPop(i, j) = pop(i, j) + alpha * r1 * (Best(j) - pop(i, j));
            else
                newPop(i, j) = pop(i, j) - beta * r2 * (Worst(j) - pop(i, j));
            end

            % PO exploitation
            newPop(i, j) = newPop(i, j) + gamma * (mean(pop(:, j)) - pop(i, j));
        end

        % Boundary Handling
        newPop(i, :) = max(newPop(i, :), lowerBounds);
        newPop(i, :) = min(newPop(i, :), upperBounds);

        % Evaluate
        newFitness(i) = objFcn(newPop(i, :));

        % Selection
        if newFitness(i) < fitness(i)
            pop(i, :) = newPop(i, :);
            fitness(i) = newFitness(i);
        end
    end

    % Save Best of Current Generation
    [bestFit, bestIdx] = min(fitness);
    bestFitHistory(gen) = bestFit;
    bestSolHistory(gen, :) = pop(bestIdx, :);

    % Update Hybrid factor
    gamma = gamma * 1.05; % gradually increase exploitation

    fprintf('Generation %02d | Best Cost: %.6f | Kp: %.4f | Ki: %.4f\n', ...
        gen, bestFit, bestSolHistory(gen, 1), bestSolHistory(gen, 2));
end

executionTime = toc(ticAlgo);

%% Results Summary
fprintf('\n=== Final Optimized PI Controller Gains ===\n');
fprintf('Kp: %.6f\n', bestSolHistory(end, 1));
fprintf('Ki: %.6f\n', bestSolHistory(end, 2));
fprintf('Best Cost: %.6f\n', bestFitHistory(end));
fprintf('Execution Time: %.2f seconds\n', executionTime);

%% Plot Convergence
figure;
plot(1:maxGen, bestFitHistory, 'b-', 'LineWidth', 2);
xlabel('Generation');
ylabel('Best Cost');
title('Hybrid GEO-PO Convergence for PI Controller');
grid on;

%% Save Results
results.algorithm = 'Hybrid GEO-PO';
results.bestCost = bestFitHistory(end);
results.executionTime = executionTime;
results.bestParams = bestSolHistory(end, :);

save('Hybrid_GEO_PO_PI_Results.mat', 'results');

%% ========================================================================
% Objective Function Definition
% Replace with your closed-loop system ITAE or similar.
% Example: Sphere function for demonstration
% ========================================================================
function cost = PI_Controller_Objective(x)
    global Kp Ki
Kp=x(1,1); Ki=x(1,2);
sim('Dual_Boost_71V_GEO_PO')
% CTRL_Error = Simulink.SimulationOutput
    cost = CTRL_E; 
end
