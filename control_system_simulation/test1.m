function temp_level = classifytemperature(temp)
    if temp >= 40
  temp_level = '酷热';
    elseif temp >= 30
  temp_level = '炎热';
    elseif temp >= 20 && temp <= 29
  temp_level = '舒适';
    elseif temp >= 10 && temp <= 19
  temp_level = '凉爽';
    else
  temp_level = '寒冷';
    end
end
