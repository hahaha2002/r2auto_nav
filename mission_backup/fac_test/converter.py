def format_array(array):
    converted =''
    for i in range(len(array)):
        if (array[i] != '[') and (array[i]!=']'):
            converted+= array[i]
    converted = converted.split(", ")
    converted = converted[:-1]
    for i in range(0, len(converted)):
        converted[i] = float(converted[i])
    return converted
