function epoch = v2_as_datetime(value)
if isdatetime(value)
    epoch = value;
else
    epoch = datetime(value, 'TimeZone', 'UTC');
end
if isempty(epoch.TimeZone)
    epoch.TimeZone = 'UTC';
end
epoch.Format = 'yyyy-MM-dd HH:mm:ss Z';
end
