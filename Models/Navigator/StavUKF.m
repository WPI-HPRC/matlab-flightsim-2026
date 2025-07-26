classdef StavUKF
    %STAVUKF Summary of this class goes here
    %   Detailed explanation goes here

    properties
        dynamicsFunc
        Q
        n
    end

    methods
        function obj = StavUKF(dynamicsFunc_, Q_)
            obj.dynamicsFunc = dynamicsFunc_;
            obj.Q = Q_;

            obj.n = size(obj.Q,1);
        end
        
        function [priorStruct,sigmaPtsStructPropagated] = predict(obj, stateStruct, dt, addFuncInputs)
            
            % Parse inputs
            x_prev = stateStruct.mean;
            P_prev = stateStruct.cov;
            
            % Generate sigma points
            sigmaPtsStruct = obj.generateSigmaPoints(x_prev, P_prev, 1e-3, 2, 0);
            
            % Propagate sigma points
            propagatedSigPts = zeros(size(sigmaPtsStruct.sigmaPts));
            for i = 1:2*obj.n+1
                propagatedSigPts(:,i) = obj.dynamicsFunc(sigmaPtsStruct.sigmaPts(:,i), dt, addFuncInputs);
            end

            % Recover mean and covariance
            sigmaPtsStructPropagated = sigmaPtsStruct;
            sigmaPtsStructPropagated.sigmaPts = propagatedSigPts;
            [mean, cov] = obj.recoverMeanCov(sigmaPtsStructPropagated);
            
            % Add process noise
            cov = cov + obj.Q*dt;
            
            % Construct output structure
            priorStruct.mean = mean;
            priorStruct.cov = cov;
        
        end

        function posteriorStruct = correct(obj, priorStruct, propagatedSigmaPtsStruct, measFunc, R, addFuncInputs, measurement)
            
            % Parse inputs
            x_prior = priorStruct.mean;
            P_prior = priorStruct.cov;
            
            % Generate estimated measurements from sigma points
            measurementSigmaPts = zeros(size(measFunc(propagatedSigmaPtsStruct.sigmaPts(:,1), addFuncInputs), 1), 2*obj.n+1);
            for i = 1:2*obj.n+1
                measurementSigmaPts(:,i) = measFunc(propagatedSigmaPtsStruct.sigmaPts(:,i), addFuncInputs);
            end

            % Recover mean and covariance
            measurementSigmaPtsStruct = propagatedSigmaPtsStruct;
            measurementSigmaPtsStruct.sigmaPts = measurementSigmaPts;
            [measMean, measCov] = obj.recoverMeanCov(measurementSigmaPtsStruct);
            
            % Add measurement noise
            measCov = measCov + R;
            
            % Compute cross-covariance
            Cxz = obj.computeCrossCovariance(x_prior, propagatedSigmaPtsStruct, measMean, measurementSigmaPtsStruct);
            
            % Compute Kalman Gain
            K = Cxz/measCov;

            % Correct mean and covariance
            x_posterior = x_prior + K*(measurement - measMean);
            P_posterior = P_prior - K*measCov*K';
            % Construct output structure
            posteriorStruct.mean = x_posterior;
            posteriorStruct.cov = P_posterior;
        
        end



        function sigmaPtsStruct = generateSigmaPoints(obj,x_prev,P_prev,alpha,beta,kappa)
            
            % Initialization
            lambda = alpha^2 * (obj.n + kappa) - obj.n;
            
            sigmaPoints = zeros(obj.n,2*obj.n+1);
            weightsMean = zeros(1,2*obj.n+1);
            weightsCov = zeros(1,2*obj.n+1);

            % Get sigma point offset from mean
            S = chol((obj.n + lambda)*P_prev)';
            
            % Generate Sigma Points symetrically about mean (and include mean)
            sigmaPoints(:,1) = x_prev;
            sigmaPoints(:,2:obj.n+1) = repmat(x_prev,1,obj.n) + S;
            sigmaPoints(:,obj.n+2:end) = repmat(x_prev,1,obj.n) - S;

            % Generate Weights
            weightsMean(1) = lambda/(obj.n + lambda);
            weightsMean(2:end) = 1/(2*(obj.n+lambda));

            weightsCov(1) = weightsMean(1) + (1 - alpha^2 + beta^2);
            weightsCov(2:end) = 1/(2*(obj.n+lambda));

            % Generate Output Struct
            sigmaPtsStruct.sigmaPts = sigmaPoints;
            sigmaPtsStruct.weightsMean = weightsMean;
            sigmaPtsStruct.weightsCov = weightsCov;
        end

        function [mean,cov] = recoverMeanCov(obj,sigmaPtsStruct)
            
            % Parse inputs
            sigmaPts = sigmaPtsStruct.sigmaPts;
            weightsMean = sigmaPtsStruct.weightsMean;
            weightsCov = sigmaPtsStruct.weightsCov;
            
            % Compute mean (sum of weights == 1)
            mean = sigmaPts*weightsMean';
            
            % Compute Covariance
            cov = zeros(size(sigmaPts,1));
            for i = 1:length(weightsCov)
                cov = cov + weightsCov(i)*(sigmaPts(:,i) - mean)*(sigmaPts(:,i) - mean)';
            end
        end

        function cov = computeCrossCovariance(obj, x_prior, propagatedSigmaPtsStruct, measMean, measurementSigmaPtsStruct)
            
            % Parse inputs
            sigmaPtsProp = propagatedSigmaPtsStruct.sigmaPts;
            sigmaPtsMeas = measurementSigmaPtsStruct.sigmaPts;
            weightsCov = propagatedSigmaPtsStruct.weightsCov;
            
            % Compute cross-covariance
            cov = zeros(size(sigmaPtsProp,1),size(sigmaPtsMeas,1));
            for i = 1:length(weightsCov)
                cov = cov + weightsCov(i)*(sigmaPtsProp(:,i) - x_prior)*(sigmaPtsMeas(:,i) - measMean)';
            end
        end
    end
end