fid = fopen('outputtest.txt', 'w');
if fid == -1
    error('Failed to open file.');
end

fprintf(fid, 'This is a test message.\n');
fclose(fid); % Close the file
