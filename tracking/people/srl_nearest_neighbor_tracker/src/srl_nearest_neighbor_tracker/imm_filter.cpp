/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Fabian Girrbach, Social Robotics Lab, University of Freiburg
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*  * Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <srl_nearest_neighbor_tracker/base/stl_helpers.h>
#include <srl_nearest_neighbor_tracker/ros/params.h>
#include <srl_nearest_neighbor_tracker/imm_filter.h>

#include <tf/tf.h>
#include <ros/ros.h>
#include <Eigen/LU>
#include <limits>
#include <map>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#define foreach BOOST_FOREACH


namespace srl_nnt {

IMMFilter::IMMFilter(const ros::NodeHandle& nodeHandle, const ros::NodeHandle& privateNodeHandle)
: m_nodeHandle(nodeHandle),
  m_privateNodeHandle(privateNodeHandle),
  m_immVisualization(false),
  m_sendDebugInformation(false)
{
    // TODO Initialization of filter with parameters and prefixes
    m_numberModels = Params::get<int>("number_of_models", 2);
    ROS_INFO_STREAM_NAMED("IMM", "Initializing IMM filter with " << m_numberModels << " models.");

    for (unsigned int i = 0; i < m_numberModels ; i++)
    {
        double checksum = 0.0;
        for (unsigned int j = 0; j < m_numberModels ; j++)
        {
            stringstream markovParamName;
            markovParamName << "markov_trans_prob_" << i << "_"<< j;
            m_markovTransitionProbabilities(i,j) = Params::get<double>(markovParamName.str(),0.0);
            checksum += m_markovTransitionProbabilities(i,j);
            ROS_DEBUG_STREAM("Markov transition probability from " << i << " to " << j << " set to " << m_markovTransitionProbabilities(i,j));

        }
        if(std::abs(checksum - 1.0) > 0.01) ROS_ERROR_STREAM_NAMED("IMM", "Markov transition probabilities have to sum up to one!");
    }

    for (unsigned int i = 0; i < m_numberModels ; i++)
    {
        stringstream currrentParameterPrefix;
        currrentParameterPrefix << "IMM" << i << Params::get<string>("prefix_model_parameter","_");
        EKF::Ptr ekf (new EKF(currrentParameterPrefix.str()));
        m_kalmanFilters.push_back(ekf);
    }

    privateNodeHandle.getParam("use_imm_visualization", m_immVisualization);
    if (m_immVisualization)
        m_immVisualizationPublisher = m_privateNodeHandle.advertise<visualization_msgs::Marker>("imm_visualization", 10);
    privateNodeHandle.getParam("send_imm_debug_information", m_sendDebugInformation);
    if(m_sendDebugInformation){
        m_immDebugPublisher = m_privateNodeHandle.advertise<spencer_tracking_msgs::ImmDebugInfos>("imm_debug_infos",10);
        m_debugMessages.header.seq = 0;
        m_debugMessages.header.frame_id = "odom";
    }


}


void IMMFilter::setTransitionMatrix(const StateVector& x,const double deltaT)
{
    ROS_DEBUG_NAMED("IMM", "Setting IMM transitions Matrix to all Kalman filters");
    foreach(EKF::Ptr ekf, m_kalmanFilters) {
        ekf->setTransitionMatrix(x,deltaT);
    }
}


FilterState::Ptr IMMFilter::initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity)
{
    IMMState::Ptr immState (new IMMState);
    ROS_DEBUG_NAMED("IMM", "New IMM state created");

    foreach(EKF::Ptr ekf, m_kalmanFilters) {
        //create new hypothesis for each model
        IMMHypothesis::Ptr hypothesis (new IMMHypothesis);
        ekf->initializeTrackState(hypothesis, observation, initialVelocity);
        //initialize probability of hypothesis equally depending on number of models
        hypothesis->m_probability = 1.0/(double) m_numberModels;
        immState->m_hypotheses.push_back(hypothesis);
        //ROS_INFO_STREAM("Initialize ekf with hypothesis C" << hypothesis->C() << " x " << hypothesis->x());
        m_debugMessages.header.frame_id = "odom";

    }

    // Setting current state to first hypothesis should be therefore the most probable model
    immState->m_currentHypothesis = immState->m_hypotheses.front();
    immState->m_currentHypothesisIdx =0;
    updateStateEstimate(immState);

    return immState;
}



FilterState::Ptr IMMFilter::initializeTrackStateFromLogicInitiator(InitiatorCandidate::Ptr candidate)
{
    IMMState::Ptr immState (new IMMState);
    ROS_DEBUG_NAMED("IMM", "New IMM state created");

    foreach(EKF::Ptr ekf, m_kalmanFilters) {
        // create new hypothesis for each model
        IMMHypothesis::Ptr hypothesis (new IMMHypothesis);
        hypothesis->m_x = candidate->state->x();
        hypothesis->m_C = candidate->state->C();

        // initialize probability of hypothesis equally depending on number of models
        hypothesis->m_probability = 1.0/(double) m_numberModels;
        immState->m_hypotheses.push_back(hypothesis);

        //ROS_INFO_STREAM("Initialize ekf with hypothesis C" << hypothesis->C() << " x " << hypothesis->x());
    }

    //Setting current state to first hypothesis should be therefore the most probable model
    immState->m_currentHypothesis = immState->m_hypotheses.front();
    immState->m_currentHypothesisIdx =0;
    updateStateEstimate(immState);
    return immState;
}


void IMMFilter::predictTrackState(FilterState::Ptr state, double deltatime)
{
    sendDebugInformation();
    ROS_DEBUG_NAMED("IMM", "Predict IMM track states!");

    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);
    doMixing(imm);

    // Do the traditional kalman filter predictions
    for (int i = 0 ; i < m_kalmanFilters.size(); i++)
    {
        m_kalmanFilters.at(i)->predictTrackState(imm->m_hypotheses.at(i),deltatime);
        //ROS_WARN_STREAM("Prediction for hypothesis " << i << "with probability  " << imm->m_hypotheses.at(i)->m_probability << " Cov =  \n" << imm->m_hypotheses.at(i)->C()
        //      << "\n CovP =  \n" << imm->m_hypotheses.at(i)->Cp() << "\n Transition Matrix A \n" << m_kalmanFilters.at(i)->m_A);

    }
    mixPredictions(imm);
}


void IMMFilter::mixPredictions(IMMState::Ptr state)
{
    state->m_xp = StateVector::Zero();
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        state->m_xp += hyp->m_xp * hyp->m_probability;
    }

    state->m_Cp = StateMatrix::Zero();
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        StateVector diff = hyp->m_xp - state->m_xp;
        state->m_Cp += (hyp->m_Cp + (diff * diff.transpose())) * hyp->m_probability;
    }
}


void IMMFilter::updateMatchedTrack(FilterState::Ptr state, Pairing::ConstPtr pairingTracker)
{
    // cast to IMM state
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);

    // Do the traditional kalman filter updating
    for (int i = 0 ; i < m_kalmanFilters.size(); i++)
    {
        //get kalman filter state for current hypothesis
        IMMHypothesis::Ptr hyp = imm->m_hypotheses.at(i) ;

        // Create a new pairing for later update
        Pairing::Ptr pairing( new Pairing );

        pairing->observation = pairingTracker->observation;
        pairing->validated = pairingTracker->validated;

        // Calculate innovation v and inverse of innovation covariance S
        pairing->v = pairing->observation->z - hyp->zp();
        ObsMatrix S = hyp->H() * hyp->Cp() * hyp->H().transpose() + pairing->observation->R;
        Eigen::FullPivLU<ObsMatrix> lu(S);
        double ln_det_S = log(lu.determinant());

        // Calculate inverse of innovation covariance if possible
        const double MATRIX_LN_EPS = -1e8;

        if (ln_det_S > MATRIX_LN_EPS) {
            pairing->Sinv = lu.inverse();
            pairing->d = (pairing->v.transpose() * pairing->Sinv * pairing->v)(0,0);
            pairing->singular = pairing->d < 0.0;
        }
        else {
            pairing->Sinv = ObsMatrix::Constant(OBS_DIM, OBS_DIM, numeric_limits<double>::quiet_NaN());
            pairing->d = numeric_limits<double>::quiet_NaN();
            pairing->singular = true;

            ROS_WARN_STREAM("Singular pairing encountered!\nTrack  measurement prediction:\n" << state->zp() << "\nTrack covariance prediction:\n" << hyp->Cp()
                            << "\nObservation " << pairing->observation->id << " mean:\n" << pairing->observation->z << "\nObservation covariance:\n" << pairing->observation->R
                            << "\nH:\n" << hyp->H() << "\nS:\n" << S << "\nR:\n" << pairing->observation->R);
        }

        // update kalman filter with pairing for current filter
        ROS_DEBUG_NAMED("IMM", "Updating kalman filter of IMM");
        m_kalmanFilters.at(i)->updateMatchedTrack(hyp, pairing);

        ROS_DEBUG_NAMED("IMM", "Calculating likelihood");

        // save likelihood for hypothesis
        imm->m_hypotheses.at(i)->m_likelihood = calcLikelihood(pairing->d, lu.determinant());
    }

    modeProbabilityUpdate(imm);
    updateStateEstimate(imm);
    getDebugInformation(imm,pairingTracker);
    updateCurrentHypothesis(imm, pairingTracker->track);
}


double IMMFilter::calcLikelihood(double d, double detS)
{
    //REVIEW showed that this is wrong: NMODELS should not be here and also the square of mahlanobis distance d is shit
    //likelihood = exp(-(d * d) / 2.0) / (sqrt(pow((2 * M_PI), N_MODELS) * detS ));
    //ROS_INFO_STREAM("Exp likelihood " << likelihood);
    //Java implementation does it like this--> we can get rid of the pi factor because its just normalizing and constant for all hypotheses
    double likelihood = exp(-d/2.0) / sqrt(detS);
    return likelihood;
}


void IMMFilter::updateOccludedTrack(FilterState::Ptr state)
{
    // cast to IMM state
    IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(state);

    // Do the traditional kalman filter updating
    for (int i = 0 ; i < m_kalmanFilters.size(); i++)
    {
        IMMHypothesis::Ptr hyp = imm->getHypothesis(i);
        m_kalmanFilters.at(i)->updateOccludedTrack(hyp);
        //ROS_WARN_STREAM("Occluded update for hypothesis " << i << " Cov =  \n" << hyp->C());
    }

    updateStateEstimate(imm);
    //ROS_ERROR_STREAM("Occluded update mixed Cov= \n" << imm->C());
}


void IMMFilter::doMixing(IMMState::Ptr state)
{
    ROS_DEBUG_NAMED("IMM", "Model mixing");
    computeMixingProbabilities(state);
    computeMixedMean(state);
    computeMixedCovariance(state);
    state->useMixedValues();
}


void IMMFilter::computeMixingProbabilities(IMMState::Ptr state) {

    //ROS_DEBUG_NAMED("IMM", "Before Loop");
    for( int j = 0; j < state->getHypotheses().size(); j++ )
    {
        //FIXME Matrix representation should be possible
        double normalizer = 0.0;
        IMMHypothesis::Ptr hyp = state->getHypothesis(j);

        // Calculate normalizer which is constant for one target hypothesis
        for( int i = 0; i < state->m_hypotheses.size(); i++ )
        {
            normalizer += m_markovTransitionProbabilities(i,j) * state->getHypothesis(i)->getProbability();
        }
        hyp->m_cNormalizer = normalizer;

        // Calculate Mixed Probability for each hypothesis
        for( int i = 0; i < state->m_hypotheses.size(); i++ )
        {
            state->m_mixingProbabilities(i,j) = m_markovTransitionProbabilities(i,j) * state->getHypothesis(i)->getProbability() / hyp->m_cNormalizer;
            ROS_DEBUG_STREAM_NAMED("IMM", "Mixing probability (" << i<< ";" << j << ")=" << state->m_mixingProbabilities(i,j));
        }
    }
}


void IMMFilter::computeMixedMean(IMMState::Ptr state)
{
    // Compute mixed initial condition for all hypothesis
    for( int j = 0; j < state->getHypotheses().size(); j++ )
    {
        state->getHypothesis(j)->m_xMixed = StateVector::Zero();
        for( int i = 0; i < state->getHypotheses().size(); i++ )
        {
            state->getHypothesis(j)->m_xMixed += state->getHypothesis(i)->x() * state->m_mixingProbabilities(i,j);
        }
        ROS_DEBUG_STREAM_NAMED("IMM", "Unmixed Mean x = " << state->getHypothesis(j)->x() << " Mixed Mean x = " << state->getHypothesis(j)->xMixed() );
    }
}


void IMMFilter::computeMixedCovariance(IMMState::Ptr state)
{
    // Compute mixed covariance taken from book p.456
    // Matrix operations in java are quite strange
    for( int j = 0; j < state->getHypotheses().size(); j++ )
    {
        state->getHypothesis(j)->m_CMixed = StateMatrix::Zero();
        for( int i = 0; i < state->getHypotheses().size(); i++ )
        {
            StateVector diff = state->getHypothesis(i)->x() - state->getHypothesis(j)->xMixed();
            state->getHypothesis(j)->m_CMixed += state->m_mixingProbabilities(i,j) * (state->getHypothesis(i)->C()
                    + diff * diff.transpose());
        }
        ROS_DEBUG_STREAM_NAMED("IMM", "Unmixed Cov C = " << state->getHypothesis(j)->C() << " Mixed Cov Cmixed = " << state->getHypothesis(j)->CMixed() );
    }
}


void IMMFilter::updateStateEstimate(IMMState::Ptr state)
{
    state->m_x = StateVector::Zero();
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        state->m_x += hyp->m_x * hyp->m_probability;
    }

    state->m_C = StateMatrix::Zero();
    for (int i = 0; i < state->m_hypotheses.size(); i++)
    {
        IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
        StateVector diff = hyp->m_x - state->m_x;
        state->m_C += (hyp->m_C + (diff * diff.transpose())) * hyp->m_probability;
    }
}





    void IMMFilter::updateCurrentHypothesis(IMMState::Ptr state, Track::Ptr track)
    {
        ROS_DEBUG_NAMED("IMM", "Updating current hypothesis");

        double bestHypothesis = 0.0;
        unsigned int bestHypothesisIdx = 0;
        for (unsigned int i = 0; i < state->m_hypotheses.size(); i++)
        {
            if (state->getHypothesis(i)->m_probability > bestHypothesis)
            {
                bestHypothesis = state->getHypothesis(i)->m_probability;
                bestHypothesisIdx = i;
            }
        }

        if (state->m_currentHypothesis != state->getHypothesis(bestHypothesisIdx))
            ROS_ERROR_STREAM_NAMED("IMM", "Best Hypothesis switched to model "<< bestHypothesisIdx);

        state->m_currentHypothesis = state->getHypothesis(bestHypothesisIdx);
        state->m_currentHypothesisIdx =bestHypothesisIdx;
        track->model_idx = bestHypothesisIdx;
        //track->state = state->m_currentHypothesis;
    }


    void IMMFilter::modeProbabilityUpdate(IMMState::Ptr state)
    {
        ROS_DEBUG_NAMED("IMM", "Updating mode probability");

        double normalizer = 0.0;
        for (int i = 0; i < state->m_hypotheses.size(); i++)
        {
            IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
            normalizer += hyp->m_likelihood * hyp->m_cNormalizer;
        }
        for (int i = 0; i < state->m_hypotheses.size(); i++)
        {
            IMMHypothesis::Ptr hyp = state->m_hypotheses.at(i);
            hyp->m_probability = hyp->m_likelihood*hyp->m_cNormalizer / normalizer;
            ROS_DEBUG_STREAM_NAMED("IMM", "Mode Probability of hypothesis " << i << "is " << hyp->m_probability);
        }
    }


    void IMMFilter::visualizeFilterProperties(const ros::Time& time, const string& trackerFrame, const Tracks tracks)
    {
        // check if visualization is enabled
        if (m_immVisualization)
        {
            // Create marker lists for vertical and horizontal probability bars
            visualization_msgs::Marker marker_top, marker_side;
            marker_top.header.stamp = marker_side.header.stamp =time;
            marker_top.header.frame_id = marker_side.header.frame_id =trackerFrame;
            marker_top.id = marker_side.id =0;
            marker_top.scale.x = marker_side.scale.x =0.1;
            marker_top.pose.orientation.w = marker_side.pose.orientation.w =1.0;
            marker_top.action = marker_side.action =visualization_msgs::Marker::ADD;
            marker_top.type = marker_side.type =visualization_msgs::Marker::LINE_LIST;
            marker_top.lifetime = marker_side.lifetime =ros::Duration();
            marker_top.ns = "imm_probabilites_vertical";
            marker_side.ns = "imm_probabilites_horizontal";

            // SRL People Tracker colors
            const size_t NUM_COLOR = 10, NUM_BW = 4;
            const unsigned int rainbow_colors[NUM_COLOR + NUM_BW] = {
                                                                     0xaf1f90, 0x000846, 0x00468a, 0x00953d, 0xb2c908, 0xfcd22a, 0xffa800, 0xff4500, 0xe0000b, 0xb22222,
                                                                     0xffffff, 0xb8b8b8, 0x555555, 0x000000
            };
            unsigned int id =0;
            foreach(Track::Ptr track, tracks){
                IMMState::Ptr imm = boost::dynamic_pointer_cast<IMMState>(track->state);

                const double spacer = 0.4;
                double offset = (imm->getHypotheses().size()-1)/2.0 * (-1*spacer);
                std_msgs::ColorRGBA color ;

                double x = track->state->x()(STATE_X_IDX);
                double y = track->state->x()(STATE_Y_IDX);


                double vx = track->state->x()(STATE_VX_IDX);
                double vy = track->state->x()(STATE_VY_IDX);
                double yaw =atan2(vy, vx);

                // get bar values for each hypothesis and add them to line list
                for (int i =0; i < imm->getHypotheses().size(); i++)
                {
                    //unsigned int rgb= spencer_colors[i*NUM_SRL_COLOR_SHADES];
                    unsigned int rgb= rainbow_colors[i];

                    // get color
                    color.r = ((rgb >> 16) & 0xff) / 255.0f;
                    color.g = ((rgb >> 8)  & 0xff) / 255.0f;
                    color.b = ((rgb >> 0)  & 0xff) / 255.0f;
                    color.a = 0.8;
                    marker_top.colors.push_back(color);
                    marker_top.colors.push_back(color);
                    marker_side.colors.push_back(color);
                    marker_side.colors.push_back(color);


                    // set position and orientation for horizontal and vertical bar
                    geometry_msgs::Point p;
                    p.x = x + cos(yaw+ M_PI_2)* offset;
                    p.y = y + sin(yaw+ M_PI_2)* offset;
                    p.z = 1.8;
                    marker_top.points.push_back(p);
                    marker_side.points.push_back(p);


                    p.z += imm->getHypothesis(i)->getProbability();
                    marker_top.points.push_back(p);

                    p.x += cos(yaw) * imm->getHypothesis(i)->getProbability();
                    p.y += sin(yaw) * imm->getHypothesis(i)->getProbability();
                    p.z = 1.8;
                    marker_side.points.push_back(p);

                    offset += spacer;

                }

                // for each track visualize most probable model as colored dot
                visualization_msgs::Marker activModelMarker;
                activModelMarker.header.stamp = time; ;
                activModelMarker.header.frame_id = trackerFrame;
                activModelMarker.id =   time.toNSec() + id++;
                activModelMarker.action =       visualization_msgs::Marker::ADD;
                activModelMarker.type =     visualization_msgs::Marker::SPHERE;
                activModelMarker.lifetime =     ros::Duration().fromSec(20);
                activModelMarker.ns =       "imm_active_model_path";
                activModelMarker.scale.x = activModelMarker.scale.y = activModelMarker.scale.z = 0.1;
                unsigned int rgb= rainbow_colors[imm->getCurrentHypothesisIndex()];

                color.r = ((rgb >> 16) & 0xff) / 255.0f;
                color.g = ((rgb >> 8)  & 0xff) / 255.0f;
                color.b = ((rgb >> 0)  & 0xff) / 255.0f;
                color.a = 0.8;

                activModelMarker.color = color;
                activModelMarker.pose.position.x = x;
                activModelMarker.pose.position.y = y;
                m_immVisualizationPublisher.publish(activModelMarker);
            }
            m_immVisualizationPublisher.publish(marker_top);
            m_immVisualizationPublisher.publish(marker_side);
        }
    }


    void IMMFilter::getDebugInformation(IMMState::Ptr state, Pairing::ConstPtr pairingTracker)
    {
        //calculate innovation from Mixed state estimates
        if (m_sendDebugInformation)
        {
            ROS_WARN_STREAM("packing new debug message");
            spencer_tracking_msgs::ImmDebugInfo debug;
            debug.track_id = pairingTracker->track->id;
            debug.CXX = state->m_C(STATE_X_IDX, STATE_X_IDX);
            debug.CYY = state->m_C(STATE_Y_IDX, STATE_Y_IDX);
            debug.CpXX = state->m_Cp(STATE_X_IDX, STATE_X_IDX);
            debug.CpYY = state->m_Cp(STATE_Y_IDX, STATE_Y_IDX);
            ObsVector innovation = state->m_xp.head(OBS_DIM) - pairingTracker->observation->z;
            debug.innovation = innovation.norm();
            for (int i = 0; i < state->m_hypotheses.size(); i++)
            {
                debug.modeProbabilities.push_back(state->m_hypotheses.at(i)->getProbability());
            }
            m_debugMessages.infos.push_back(debug);
        }
    }

    void IMMFilter::sendDebugInformation()
    {
        //calculate innovation from Mixed state estimates
        if (m_sendDebugInformation && m_debugMessages.infos.size() > 0){
            m_debugMessages.header.stamp = ros::Time::now();
            m_debugMessages.header.seq++;
            ROS_WARN_STREAM("Publishing " << m_debugMessages.infos.size() << " info messages");
            m_immDebugPublisher.publish(m_debugMessages);
            m_debugMessages.infos.clear();
        }

    }


}


