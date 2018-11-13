_textures = {}

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

for _, filename in ipairs(_FILES) do

   -- print(filename)
   
   local scene = ProcessFile(filename)

   -- for _, resType in ipairs({'materials', 'textures', 'meshes'}) do
   --    for id, data in ipairs(scene[resType]) do
   --       -- print(TableToString(data, string.format("%s[%s]", resType, id - 1), "\t"))
   --    end
   -- end

   _textures[filename] = scene.textures[1].path
   
   CloseFile(filename)
end

--
if _PARAMS.out then
   print(string.format("Writing to %s", _PARAMS.out))
   OutputLua(_textures, _PARAMS.out)

else
   print(TableToString(_textures))
   
end
