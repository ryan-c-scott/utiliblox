using UnityEngine;
using UnityEditor;

using System.Text;
using System.IO;
using System.Text.RegularExpressions;

using System.Collections.Generic;

public class AssetExporterMenu
{

    delegate void SuccessDelegate();
    
    static void ProcessSampler( string exportPath, StringBuilder builder, Material mat, string unityName, string vlName, SuccessDelegate successCallback=null )
    {
        if( !mat.HasProperty( unityName ) )
            return;

        Texture tex = mat.GetTexture( unityName );

        if( tex != null ) {
            string texPath = AssetDatabase.GetAssetOrScenePath( tex );
            string outPath = exportPath + Path.GetFileName( texPath );

            // Doctor out path to ensure that normal maps are handled correctly
            if( vlName == "normalMap" || vlName == "detailNormalMap" ) {
                if( !outPath.Contains( "normal" ) ) {
                    string ext = Path.GetExtension( outPath );
                    outPath = outPath.Substring( 0, outPath.Length - ext.Length ) + "_normal" + ext;
                }
            }
            
            //Copy texture to export path
            File.Copy( texPath, outPath, true );
            
            builder.Append( "sampler\n{\n" );
            builder.Append( string.Format( "   name: \"{0}\"\n", vlName ) );
            builder.Append( string.Format( "   map: \"{0}\"\n", Regex.Replace( outPath, ".*?/content/", string.Empty ) ) );
            builder.Append( "}\n\n" );

            if( successCallback != null ) {
                successCallback();
            }
        }
    }

    static void ProcessUVOffsetScale( StringBuilder builder, Material mat, string unityName, string vlName )
    {
        if( !mat.HasProperty( unityName ) )
            return;

        Vector2 offset = mat.GetTextureOffset( unityName );
        Vector2 scale = mat.GetTextureScale( unityName );
        
        if( offset == Vector2.zero && scale == Vector2.one )
            return;
        
        builder.Append( "uniform\n{\n" );
        builder.Append( "   name: \"" + vlName + "\"\n" );
        builder.Append( "   vec4\n   {\n" );
        builder.Append( string.Format( "      x: {0}\n", scale.x ) );
        builder.Append( string.Format( "      y: {0}\n", scale.y ) );
        builder.Append( string.Format( "      z: {0}\n", offset.x ) );
        builder.Append( string.Format( "      w: {0}\n", offset.y ) );
        builder.Append( "   }\n" );
        builder.Append( "}\n\n" );
    }
    
    static void MaterialToProtobuf( string exportPath, Material mat )
    {
        List< int > features = new List< int >();
        
        StringBuilder proto = new StringBuilder();
        proto.Append( "render_class: \"Normal\"\n" );
        proto.Append( "shader: \"shaders/model.shader\"\n" );

        ProcessSampler( exportPath, proto, mat, "_MainTex", "albedoMap" );
        ProcessSampler( exportPath, proto, mat, "_BumpMap", "normalMap", ()=>{ features.Add( 3 ); } );
        ProcessSampler( exportPath, proto, mat, "_OcclusionMap", "occlusionMap" );
        ProcessSampler( exportPath, proto, mat, "_DetailAlbedoMap", "detailAlbedoMap", ()=>{ features.Add( 4 ); } );
        ProcessSampler( exportPath, proto, mat, "_DetailNormalMap", "detailNormalMap", ()=>{ features.Add( 5 ); } );

        // TODO:  Support attributes such as scale
        // .Also handle detecting appropriate shader
        ProcessUVOffsetScale( proto, mat, "_MainTex", "primaryUVScaleOffset" );
        ProcessUVOffsetScale( proto, mat, "_DetailAlbedoMap", "secondaryUVScaleOffset" );

        foreach( int featureId in features ) {
            proto.Append( string.Format( "feature: {0}\n", featureId ) );
        }
        
        Debug.LogWarning( proto );
        File.WriteAllText( exportPath + mat.name + ".material", proto.ToString() );
    }

    [ MenuItem( "Tools/Export Selected Asset" ) ]
    private static void ExportAsset()
    {
        GameObject obj = Selection.activeObject as GameObject;

        string path = AssetDatabase.GetAssetPath( Selection.activeObject );
        string exportPath = "/Users/ryan/vl/content/test/";

        //Copy mesh asset to target
        // TODO: Look into potentially triggering Blender to a conversion to collada
        File.Copy( path, exportPath + Path.GetFileName( path ), true );
        
        foreach( MeshRenderer renderer in obj.GetComponentsInChildren< MeshRenderer >( true ) ) {
            foreach( Material mat in renderer.sharedMaterials ) {
                MaterialToProtobuf( exportPath, mat );
            }
        }
    }
}
