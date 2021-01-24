function Jh = eval_Jh(q)
    [~, ~, ~, l1, ~, ~, ~] = set_parameters();
    q1 = q(1);

    Jh = reshape([l1.*cos(q1),-l1.*sin(q1),0.0,0.0,0.0,0.0],[2,3]);
end