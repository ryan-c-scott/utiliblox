function TableToString(tbl, label, pad)
   local out = ''
   if not pad then
      pad = ''
   end

   if label then
      out = string.format("%s%s\n", pad, label)
   end

   for k,v in pairs(tbl) do
      out = string.format("%s\t%s%s:\t%s\n", out, pad, k, v)
   end
   
   return out
end

print(TableToString(_PARAMS, "Parameters:"))

for _, filename in ipairs(_FILES) do

   print(filename)
   
   local scene = ProcessFile(filename)

   for _, resType in ipairs({'materials', 'textures', 'meshes'}) do
      for id, data in ipairs(scene[resType]) do
         print(TableToString(data, string.format("%s[%s]", resType, id - 1), "\t"))
      end
   end
   
   CloseFile(filename)
end

--
if _PARAMS.out then
   print(string.format("Here's where data would get sent to %s", _PARAMS.out))
   print(json.to_string(_PARAMS))
   OutputJson(_PARAMS, _PARAMS.out)
end
