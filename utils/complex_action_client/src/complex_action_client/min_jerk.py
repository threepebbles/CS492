from numpy import *
import sys, copy

#[trj, psg] = min_jerk(pos, dur, vel, acc, psg)
#
# Compute minimum-jerk trajectory through specified points
#
# INPUTS:
# pos: NxD array with the D-dimensional coordinates of N points
# dur: number of time steps (integer)
# vel: 2xD array with endpoint velocities, [] sets vel to 0
# acc: 2xD array with endpoint accelerations, [] sets acc to 0
# psg: (N-1)x1 array of via-point passage times (between 0 and dur);
#      [] causes optimization over the passage times
#
# OUTPUTS
# trj: dur x D array with the minimum-jerk trajectory
# psg: (N-1)x1 array of passage times
#
# This is an implementation of the algorithm described in:
#  Todorov, E. and Jordan, M. (1998) Smoothness maximization along
#  a predefined path accurately predicts the speed profiles of
#  complex arm movements. Journal of Neurophysiology 80(2): 696-714
# The paper is available online at www.cogsci.ucsd.edu/~todorov

# Copyright (C) Emanuel Todorov, 1998-2006

# Converted to Python and modified by Daehyung Park

class min_jerk_traj():
    def __init__(self):
        self.TOL = 1e-6

    def min_jerk(self, pos, dur, vel=None, acc=None, psg=None):

        N,D = pos.shape  # points x dimension

        if vel == None:  # default endpoint velocity is 0
            vel = zeros((2,D))

        if acc == None:  # default endpoint acceleration is 0
            acc = zeros((2,D))

        t0 = array([0, dur])

        if psg == None:  # passage times unknown, optimize
            if N > 2:
                print "Under construction!!"
                ## psg  = array(arange(dur/(N-1), dur-dur/(N-1) + self.TOL, dur/(N-1))).T
                ## func = mjCOST(psg, pos, vel, acc, t0)
                ## psg  = fminsearch(func, psg)
            else:
                psg = None

        trj = self.mjTRJ(psg, pos, vel, acc, t0, dur); 

        return trj, psg


    #########################################################################
    #  compute jerk cost
    ## function J = mjCOST(t, x, v0, a0, t0)

    ## N = size(x,1); D = size(x,2);

    ## [v,a] = mjVelAcc(t, x, v0, a0, t0);
    ## aa = [a0(1,:);a;a0(2,:)]; aa0 = aa(1:N-1,:); aa1 = aa(2:N,:);
    ## vv = [v0(1,:);v;v0(2,:)]; vv0 = vv(1:N-1,:); vv1 = vv(2:N,:);
    ## tt = [t0(1);t;t0(2)]; T = diff(tt)*ones(1,D);
    ## xx0 = x(1:N-1,:); xx1 = x(2:N,:);

    ## j=3.*(3.*aa0.^2.*T.^4-2.*aa0.*aa1.*T.^4+3.*aa1.^2.*T.^4+24.*aa0.*T.^3.*vv0-...
    ##   16.*aa1.*T.^3.*vv0 + 64.*T.^2.*vv0.^2 + 16.*aa0.*T.^3.*vv1 -             ...
    ##   24.*aa1.*T.^3.*vv1 + 112.*T.^2.*vv0.*vv1 + 64.*T.^2.*vv1.^2 +            ...
    ##   40.*aa0.*T.^2.*xx0 - 40.*aa1.*T.^2.*xx0 + 240.*T.*vv0.*xx0 +             ...
    ##   240.*T.*vv1.*xx0 +240.*xx0.^2 - 40.*aa0.*T.^2.*xx1 + 40.*aa1.*T.^2.*xx1- ...
    ##   240.*T.*vv0.*xx1 - 240.*T.*vv1.*xx1 - 480.*xx0.*xx1 + 240.*xx1.^2)./T.^5;

    ## J = sum(sum(abs(j)));


    #########################################################################
    #  compute trajectory
    def mjTRJ(self, tx, x, v0, a0, t0, P):

        N,D = x.shape

        if tx != None:
            ## v,a = mjVelAcc(tx, x, v0, a0, t0)
            ## aa = vstack((a0[0,:],a,a0[1,:]))
            ## vv = vstack((v0[0,:],v,v0[1,:])) 
            ## t  = vstack((t0[0],tx,t0[1]))
            print "Under construction!!"        
        else:
            aa = a0
            vv = v0
            tt = t0

        X  = []
        ii = 0;
        for i in range(P):
            t = float(i)/float(P-1)*(t0[1]-t0[0]) + t0[0]

            if t>tt[ii+1]: ii = ii+1 
            T = (tt[ii+1]-tt[ii])*ones((1,D))
            t = (t-tt[ii])*ones((1,D))

            aa0 = aa[ii,:]; aa1 = aa[ii+1,:]
            vv0 = vv[ii,:]; vv1 = vv[ii+1,:]
            xx0 = x[ii,:];  xx1 = x[ii+1,:]

            X.append(\
                     aa0*t*2./2 + t*vv0 + xx0 + t**4.*(3.*aa0*T**2./2 - aa1*T**2 + \
                    8.*T*vv0 + 7.*T*vv1 + 15.*xx0 - 15.*xx1)/T**4 + \
                    t**5.*(-(aa0*T**2)/2 + aa1*T**2./2 - 3*T*vv0 - 3.*T*vv1 - 6.*xx0+ \
                    6*xx1)/T**5 + t**3.*(-3.*aa0*T**2./2 + aa1*T**2./2 - 6.*T*vv0 -    \
                    4.*T*vv1 - 10.*xx0 + 10.*xx1)/T**3)

                ## ((aa0*t)**2)/2 + 
                ## t*vv0 + xx0 + 
                ## (t**4)*(((3*aa0*T)**2)/2 - (aa1*T)**2 + \
                ## (8*T*vv0 + 7*T*vv1 + 15*xx0 - 15*xx1)/T)**4 +                          \
                ## ((t**5)*(-((aa0*T)**2)/2 + ((aa1*T)**2)/2 - 3*T*vv0 - 3*T*vv1 - 6*xx0+ \
                ## 6*xx1)/T)**5 + ((t**3)*(((-3*aa0*T)**2)/2 + ((aa1*T)**2)/2 - 6*T*vv0 - \
                ## 4*T*vv1 - 10*xx0 + 10*xx1)/T)**3)

        return array(X)

    #########################################################################
    #  compute intermediate velocities and accelerations
    ## function [v,a] = mjVelAcc(t, x, v0, a0, t0)

    ## N = size(x,1); D = size(x,2);
    ## mat = zeros(2*N-4,2*N-4); vec = zeros(2*N-4,D); 
    ## tt = [t0(1);t;t0(2)];

    ## for i=1:2:2*N-4
    ##   ii = ceil(i/2)+1; T0 = tt(ii)-tt(ii-1); T1 = tt(ii+1)-tt(ii);
    ##   tmp = [-6/T0 			-48/T0^2 	+18*(1/T0+1/T1) ...
    ## 	 +72*(1/T1^2-1/T0^2) 	-6/T1 		+48/T1^2];
    ##   if i==1 le = 0; else le = -2; end;
    ##   if i==2*N-5 ri = 1; else ri = 3; end;
    ##   mat(i,i+le:i+ri) = tmp(3+le:3+ri);
    ##   vec(i,:) = 120*(x(ii-1,:)-x(ii,:))/T0^3 + 120*(x(ii+1,:)-x(ii,:))/T1^3; 
    ## end;

    ## for i=2:2:2*N-4
    ##   ii = ceil(i/2)+1; T0 = tt(ii)-tt(ii-1); T1 = tt(ii+1)-tt(ii);
    ##   tmp = [48/T0^2 		336/T0^3 	+72*(1/T1^2-1/T0^2) ...
    ## 	 +384*(1/T1^3+1/T0^3) 	-48/T1^2 	+336/T1^3];
    ##   if i==2 le = -1; else le = -3; end;
    ##   if i==2*N-4 ri = 0; else ri = 2; end;
    ##   mat(i,i+le:i+ri) = tmp(4+le:4+ri);
    ##   vec(i,:) = 720*(x(ii,:)-x(ii-1,:))/T0^4 + 720*(x(ii+1,:)-x(ii,:))/T1^4; 
    ## end;

    ## T0 = tt(2)-tt(1); T1 = tt(N)-tt(N-1);
    ## vec(1,:) = vec(1,:) + 6/T0*a0(1,:) + 48/T0^2*v0(1,:);
    ## vec(2,:) = vec(2,:) - 48/T0^2*a0(1,:) - 336/T0^3*v0(1,:);
    ## vec(2*N-5,:) = vec(2*N-5,:) + 6/T1*a0(2,:) - 48/T1^2*v0(2,:);
    ## vec(2*N-4,:) = vec(2*N-4,:) + 48/T1^2*a0(2,:) - 336/T1^3*v0(2,:);

    ## avav = inv(mat)*vec;
    ## a = avav(1:2:2*N-4,:); v = avav(2:2:2*N-4,:);

if __name__ == '__main__':


    pos = array([[-4.0, -4.0, -4.0],[4.0, 4.0, 4.0]])
    pos = array([[0],[1.0]])
    
    mjt = min_jerk_traj()
    
    trj, psg = mjt.min_jerk(pos, 100)
    import numpy as np
    print np.shape(trj)
    print np.shape(psg)

    aTraj = trj

    import matplotlib.pyplot as plt
    fig = plt.figure(1)
    ax = fig.add_subplot(1, 1, 1)

    length,_,_ = array(trj).shape
    plt.plot(range(length), aTraj[:,0,0], 'r-', markersize=12)
    
    ## plt.ion()
    plt.show()

    ## import hrl_lib.util as ut
    ## ut.get_keystroke('Hit a key to proceed next')   
    
    
