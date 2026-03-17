%% --- Helper: Send Status ---
function sendStatus(pub, txt)
%#codegen
msg = ros2message(pub);
msg.data = char(txt);
send(pub, msg);
fprintf("[STATUS] %s\n", txt);

end
