function e=ekf_update(e,position,c)
H=[eye(3),zeros(3,12)]; R=c.sensor.positionSigma^2*eye(3);
innovation=position-e.p; S=H*e.P*H'+R;
e.lastNIS=innovation'*(S\innovation);
if e.lastNIS>c.ekf.gnssGate, e.gnssRejected=e.gnssRejected+1; return; end
K=(e.P*H')/S; delta=K*innovation;
e.p=e.p+delta(1:3); e.v=e.v+delta(4:6);
e.q=quad.Math.normalize(quad.Math.mul(e.q,quad.Math.exp(delta(7:9))));
e.ba=e.ba+delta(10:12); e.bg=e.bg+delta(13:15);
e.omega=e.omega-delta(13:15);
A=eye(15)-K*H; P=A*e.P*A'+K*R*K'; % Joseph update.
G=eye(15); G(7:9,7:9)=eye(3)-quad.Math.skew(delta(7:9))/2;
e.P=G*P*G'; e.P=(e.P+e.P')/2;
e.gnssAccepted=e.gnssAccepted+1;
end
